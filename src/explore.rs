//! Search a fan of candidate lines from a fixed start point and rank them
//! by routing viability.
//!
//! For each candidate, we run A* on a single shared graph (built once over a
//! bbox that encloses every candidate's corridor) and score the result. Lines
//! that fail to route, or that violate a no-go zone, are rejected outright.
//! Surviving candidates are ranked by a difficulty score that combines:
//!   - excess distance over the great circle (route detours)
//!   - long-edge count (almost always lakes / glaciers / graph gaps)
//!   - max corridor deviation
//!
//! Lower score = easier line.

use crate::elevation::{ElevationClient, Profile};
use crate::geodesy::{bearing_rad, destination, haversine, LatLon};
use crate::osm::{BBox, Graph};
use crate::route::{self, RouteParams, RouteResult};
use crate::viability::{self, NoGoZone, Report};
use anyhow::Result;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Clone, Debug)]
pub struct Candidate {
    pub start: LatLon,
    pub end: LatLon,
    pub bearing_deg: f64,
    pub distance_km: f64,
}

impl Candidate {
    pub fn from_endpoints(start: LatLon, end: LatLon) -> Self {
        let raw = bearing_rad(start, end).to_degrees();
        let bearing_deg = ((raw % 360.0) + 360.0) % 360.0;
        let distance_km = haversine(start, end) / 1000.0;
        Self {
            start,
            end,
            bearing_deg,
            distance_km,
        }
    }
}

/// Post-routing filters that reject candidates the cost function alone can't
/// catch. The corridor cutoff lives in [`RouteParams`]; this struct covers the
/// rest:
/// - snap displacement: rejects candidates whose nearest graph node is too far
///   from the requested A or B (e.g. country-mode `end` falling outside the OSM
///   data bbox);
/// - routed-distance bounds: ensures the suggestion respects the user's
///   requested distance range;
/// - corridor compliance: belt-and-braces check that `max_deviation_m` did
///   not slip past the cost-closure cutoff (it shouldn't, but the post-filter
///   guards against drift if either side is later relaxed).
#[derive(Clone, Debug)]
pub struct RouteFilter {
    pub distance_min_km: Option<f64>,
    pub distance_max_km: Option<f64>,
    pub snap_tolerance_m: f64,
    pub enforce_corridor: bool,
}

impl RouteFilter {
    /// Permissive filter: only enforces snap tolerance and the corridor.
    /// Use for `between` / `plan` / `explore` where the user pinned the
    /// endpoints themselves.
    pub fn lax(snap_tolerance_m: f64) -> Self {
        Self {
            distance_min_km: None,
            distance_max_km: None,
            snap_tolerance_m,
            enforce_corridor: true,
        }
    }
}

#[derive(Debug)]
pub struct Scored {
    pub candidate: Candidate,
    pub route: RouteResult,
    pub report: Report,
    pub score: f64,
    pub elevation: Option<Profile>,
}

pub fn generate_candidates(
    start: LatLon,
    bearing_min: f64,
    bearing_max: f64,
    bearing_step: f64,
    distance_min_km: f64,
    distance_max_km: f64,
    distance_step_km: f64,
) -> Vec<Candidate> {
    let mut out = Vec::new();
    let bearings = bearing_range(bearing_min, bearing_max, bearing_step);
    let distances = float_range(distance_min_km, distance_max_km, distance_step_km);
    for &bd in &bearings {
        for &dk in &distances {
            let normalized = ((bd % 360.0) + 360.0) % 360.0;
            let end = destination(start, normalized.to_radians(), dk * 1000.0);
            out.push(Candidate {
                start,
                end,
                bearing_deg: normalized,
                distance_km: dk,
            });
        }
    }
    out
}

/// Sample diverse candidates across a bbox: random start anywhere in the box,
/// random bearing, distance picked from a uniform range. Caller filters
/// downstream by routing viability + dedup.
pub fn generate_country_candidates(
    bbox: BBox,
    n: usize,
    distance_min_km: f64,
    distance_max_km: f64,
    seed: u64,
) -> Vec<Candidate> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    let mut rng = StdRng::seed_from_u64(seed);
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let start = LatLon::new(
            rng.gen_range(bbox.lat_min..bbox.lat_max),
            rng.gen_range(bbox.lon_min..bbox.lon_max),
        );
        let bearing = rng.gen_range(0.0..360.0_f64);
        let dist = rng.gen_range(distance_min_km..distance_max_km);
        let end = destination(start, bearing.to_radians(), dist * 1000.0);
        out.push(Candidate {
            start,
            end,
            bearing_deg: bearing,
            distance_km: dist,
        });
    }
    out
}

/// Sample candidates whose start AND end both fall inside the bbox, with
/// great-circle distance in `[distance_min_km, distance_max_km]`. Picks two
/// independent uniform points in the bbox and accepts the pair if the distance
/// fits. This prevents country mode from proposing lines whose `end` falls
/// outside the OSM data — those used to be silently snapped to a node back
/// near `start`, producing the bogus "ref 390 km → real 3 km" suggestions.
pub fn generate_country_candidates_in_bbox(
    bbox: BBox,
    n: usize,
    distance_min_km: f64,
    distance_max_km: f64,
    seed: u64,
) -> Vec<Candidate> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    let mut rng = StdRng::seed_from_u64(seed);
    let mut out = Vec::with_capacity(n);
    let max_attempts = (n * 80).max(2_000);
    let mut attempts = 0;
    while out.len() < n && attempts < max_attempts {
        attempts += 1;
        let start = LatLon::new(
            rng.gen_range(bbox.lat_min..bbox.lat_max),
            rng.gen_range(bbox.lon_min..bbox.lon_max),
        );
        let end = LatLon::new(
            rng.gen_range(bbox.lat_min..bbox.lat_max),
            rng.gen_range(bbox.lon_min..bbox.lon_max),
        );
        let d_km = haversine(start, end) / 1000.0;
        if d_km < distance_min_km || d_km > distance_max_km {
            continue;
        }
        out.push(Candidate::from_endpoints(start, end));
    }
    out
}

/// Sample candidates whose start and end both fall inside a border strip
/// (within `strip_km` of any bbox edge) and on different sides of the bbox.
/// Used for "I want lines that cross the country" mode.
pub fn generate_border_to_border_candidates(
    bbox: BBox,
    n: usize,
    distance_min_km: f64,
    distance_max_km: f64,
    strip_km: f64,
    seed: u64,
) -> Vec<Candidate> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    let mut rng = StdRng::seed_from_u64(seed);
    // Convert the strip width into degrees. Approximate cos(lat) for the longitude
    // axis using the bbox midpoint — accurate enough at country scale.
    let mid_lat = (bbox.lat_min + bbox.lat_max) * 0.5;
    let strip_lat_deg = strip_km / 111.0;
    let strip_lon_deg = strip_km / (111.0 * mid_lat.to_radians().cos().max(0.1));

    let pick_on_border = |rng: &mut StdRng| -> (LatLon, u8) {
        let side: u8 = rng.gen_range(0..4);
        let (lat, lon) = match side {
            0 => (
                rng.gen_range((bbox.lat_max - strip_lat_deg)..bbox.lat_max),
                rng.gen_range(bbox.lon_min..bbox.lon_max),
            ),
            1 => (
                rng.gen_range(bbox.lat_min..bbox.lat_max),
                rng.gen_range((bbox.lon_max - strip_lon_deg)..bbox.lon_max),
            ),
            2 => (
                rng.gen_range(bbox.lat_min..(bbox.lat_min + strip_lat_deg)),
                rng.gen_range(bbox.lon_min..bbox.lon_max),
            ),
            _ => (
                rng.gen_range(bbox.lat_min..bbox.lat_max),
                rng.gen_range(bbox.lon_min..(bbox.lon_min + strip_lon_deg)),
            ),
        };
        (LatLon::new(lat, lon), side)
    };

    let mut out = Vec::with_capacity(n);
    let max_attempts = (n * 40).max(200);
    let mut attempts = 0;
    while out.len() < n && attempts < max_attempts {
        attempts += 1;
        let (start, s1) = pick_on_border(&mut rng);
        let (end, s2) = pick_on_border(&mut rng);
        if s1 == s2 {
            continue;
        }
        let d_km = haversine(start, end) / 1000.0;
        if d_km < distance_min_km || d_km > distance_max_km {
            continue;
        }
        out.push(Candidate::from_endpoints(start, end));
    }
    out
}

/// Inclusive bearing range that wraps across 0° if `max < min`. The two
/// halves [min, 360) and [0, max] are stitched together at `step` resolution.
fn bearing_range(min: f64, max: f64, step: f64) -> Vec<f64> {
    if step <= 0.0 {
        return vec![min];
    }
    if max >= min {
        return float_range(min, max, step);
    }
    // Wrap through 360°. Use a single sweep over [min, max + 360], values are
    // normalized to [0, 360) by the caller.
    float_range(min, max + 360.0, step)
}

fn float_range(min: f64, max: f64, step: f64) -> Vec<f64> {
    if step <= 0.0 || max < min {
        return vec![min];
    }
    let n = ((max - min) / step).floor() as usize;
    (0..=n).map(|i| min + (i as f64) * step).collect()
}

/// BBox that encloses the start point and all candidate endpoints, plus padding.
pub fn enclosing_bbox(start: LatLon, candidates: &[Candidate], pad_deg: f64) -> BBox {
    let mut lat_min = start.lat;
    let mut lat_max = start.lat;
    let mut lon_min = start.lon;
    let mut lon_max = start.lon;
    for c in candidates {
        lat_min = lat_min.min(c.end.lat);
        lat_max = lat_max.max(c.end.lat);
        lon_min = lon_min.min(c.end.lon);
        lon_max = lon_max.max(c.end.lon);
    }
    BBox {
        lat_min: lat_min - pad_deg,
        lat_max: lat_max + pad_deg,
        lon_min: lon_min - pad_deg,
        lon_max: lon_max + pad_deg,
    }
}

/// Combined difficulty score (lower is better).
fn score(report: &Report, ref_km: f64) -> f64 {
    let excess_ratio = (report.total_km / ref_km - 1.0).max(0.0);
    let long_edge_total_km: f64 =
        report.long_edges.iter().map(|le| le.length_m / 1000.0).sum();
    let max_dev_pct = report.max_deviation_m / 1000.0;
    100.0 * excess_ratio + 5.0 * long_edge_total_km + 8.0 * max_dev_pct
}

pub fn route_one(
    graph: &Graph,
    candidate: Candidate,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
    long_edge_threshold_m: f64,
) -> Result<Scored, (Candidate, String)> {
    let pair = graph.closest_connected_pair(candidate.start, candidate.end);
    let Some((s_idx, e_idx, sd, ed)) = pair else {
        return Err((candidate, "graph empty".into()));
    };
    if sd > filter.snap_tolerance_m || ed > filter.snap_tolerance_m {
        return Err((
            candidate,
            format!(
                "endpoint too far from graph: start {:.0} m, end {:.0} m (tolerance {:.0} m)",
                sd, ed, filter.snap_tolerance_m
            ),
        ));
    }
    match route::shortest(graph, s_idx, e_idx, candidate.start, candidate.end, params) {
        Ok(r) => {
            let ref_km = haversine(candidate.start, candidate.end) / 1000.0;
            let report = viability::analyse(
                &r,
                candidate.start,
                candidate.end,
                no_go,
                long_edge_threshold_m,
            );
            if !report.violations.is_empty() {
                return Err((
                    candidate,
                    format!(
                        "no-go: {}",
                        report
                            .violations
                            .iter()
                            .map(|v| v.zone.as_str())
                            .collect::<Vec<_>>()
                            .join(", ")
                    ),
                ));
            }
            if filter.enforce_corridor && report.max_deviation_m > params.half_width_m {
                return Err((
                    candidate,
                    format!(
                        "max deviation {:.0} m exceeds corridor half-width {:.0} m",
                        report.max_deviation_m, params.half_width_m
                    ),
                ));
            }
            if let Some(min_km) = filter.distance_min_km {
                if report.total_km < min_km {
                    return Err((
                        candidate,
                        format!(
                            "routed distance {:.1} km below minimum {:.1} km",
                            report.total_km, min_km
                        ),
                    ));
                }
            }
            if let Some(max_km) = filter.distance_max_km {
                if report.total_km > max_km {
                    return Err((
                        candidate,
                        format!(
                            "routed distance {:.1} km above maximum {:.1} km",
                            report.total_km, max_km
                        ),
                    ));
                }
            }
            let s = score(&report, ref_km);
            Ok(Scored {
                candidate,
                route: r,
                report,
                score: s,
                elevation: None,
            })
        }
        Err(e) => Err((candidate, e.to_string())),
    }
}

pub fn evaluate(
    graph: &Graph,
    candidates: Vec<Candidate>,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
    long_edge_threshold_m: f64,
    progress: Option<&AtomicUsize>,
) -> Vec<Result<Scored, (Candidate, String)>> {
    use rayon::prelude::*;
    candidates
        .into_par_iter()
        .map(|c| {
            let r = route_one(graph, c, no_go, params, filter, long_edge_threshold_m);
            if let Some(p) = progress {
                p.fetch_add(1, Ordering::Relaxed);
            }
            r
        })
        .collect()
}

/// Sort the surviving Scored entries by ascending score (easiest first).
pub fn rank(mut scored: Vec<Scored>) -> Vec<Scored> {
    scored.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());
    scored
}

/// Rank, deduplicate near-identical pairs, take top `n`. Two candidates count
/// as duplicates if both their start AND end points are within `min_sep_km`.
pub fn rank_dedup_top(
    mut scored: Vec<Scored>,
    n: usize,
    min_sep_km: f64,
) -> Vec<Scored> {
    scored.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());
    let mut out: Vec<Scored> = Vec::with_capacity(n);
    let sep_m = min_sep_km * 1000.0;
    for s in scored {
        if out.len() >= n {
            break;
        }
        let dup = out.iter().any(|kept| {
            haversine(kept.candidate.start, s.candidate.start) < sep_m
                && haversine(kept.candidate.end, s.candidate.end) < sep_m
        });
        if !dup {
            out.push(s);
        }
    }
    out
}

/// Fill in elevation profiles for each surviving candidate. Soft-fails if
/// the network call breaks — affected entries simply have `None`.
pub fn enrich_with_elevation(
    scored: &mut [Scored],
    client: &mut ElevationClient,
    samples_per_route: usize,
) {
    let total = scored.len();
    for (i, s) in scored.iter_mut().enumerate() {
        eprintln!(
            "  elevation {}/{}: bearing {:.0}°, {:.1} km",
            i + 1,
            total,
            s.candidate.bearing_deg,
            s.report.total_km
        );
        s.elevation = client.profile(&s.route.points, samples_per_route);
    }
    let _ = client.save();
}

pub fn print_leaderboard(ranked: &[Scored], start: LatLon) {
    println!("\n— Leaderboard ({} viable candidates) —", ranked.len());
    println!(
        "{:>4}  {:>7}  {:>7}  {:>7}  {:>7}  {:>6}  {:>6}  {:>6}",
        "rank", "bearing", "ref_km", "real_km", "dev_m", "asc_m", "max_m", "score"
    );
    for (i, s) in ranked.iter().enumerate() {
        let ref_km = haversine(start, s.candidate.end) / 1000.0;
        let (asc, max_alt) = match &s.elevation {
            Some(p) => (p.ascent_m, p.max_m),
            None => (f64::NAN, f64::NAN),
        };
        println!(
            "{:>4}  {:>6.0}°  {:>7.1}  {:>7.1}  {:>7.0}  {:>6}  {:>6}  {:>6.1}",
            i + 1,
            s.candidate.bearing_deg,
            ref_km,
            s.report.total_km,
            s.report.max_deviation_m,
            fmt_m(asc),
            fmt_m(max_alt),
            s.score,
        );
    }
}

fn fmt_m(v: f64) -> String {
    if v.is_nan() {
        "  —  ".to_string()
    } else {
        format!("{:.0}", v)
    }
}
