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
use crate::osm::{BBox, EdgeData, Graph, MODE_FERRY};
use crate::route::{self, RouteParams, RouteResult};
use crate::shape::{place_shape, unit_max_radius, unit_shape_perimeter, ShapeKind};
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

#[derive(Debug, Clone)]
pub struct Scored {
    pub candidate: Candidate,
    pub route: RouteResult,
    pub report: Report,
    pub score: f64,
    pub elevation: Option<Profile>,
    /// Full-width corridor (metres) actually used to produce this route.
    /// Equals the user request unless the search loop widened it on retry to
    /// recover from empty batches; the UI surfaces this as a "looser corridor"
    /// badge so the user can tell when persistence kicked in.
    pub corridor_used_m: f64,
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

/// Combined difficulty score (lower is better). Ascent is in metres of
/// cumulative elevation gain across the route; pass 0.0 (or `None` upstream)
/// to fall back to a flat scoring that ignores it.
fn score(report: &Report, ref_km: f64, ascent_m: f64) -> f64 {
    let excess_ratio = (report.total_km / ref_km - 1.0).max(0.0);
    let long_edge_total_km: f64 = report
        .long_edges
        .iter()
        .map(|le| le.length_m / 1000.0)
        .sum();
    let max_dev_pct = report.max_deviation_m / 1000.0;
    // Per-km ascent puts a 100 km flat ride next to a 100 km hilly ride on a
    // comparable footing with the detour term — 10 m/km is +0.5 score, 50 m/km
    // is +2.5.
    let per_km = if report.total_km > 0.0 {
        ascent_m / report.total_km
    } else {
        0.0
    };
    100.0 * excess_ratio + 5.0 * long_edge_total_km + 8.0 * max_dev_pct + 0.05 * per_km
}

/// Recompute `Scored.score` after elevation has been filled in. Country and
/// shape-search call this so the final ranking picks the lowest-ascent route
/// that still satisfies the corridor constraints.
pub fn rescore_with_elevation(s: &mut Scored) {
    let ref_km = haversine(s.candidate.start, s.candidate.end) / 1000.0;
    let ascent = s.elevation.as_ref().map(|e| e.ascent_m).unwrap_or(0.0);
    s.score = score(&s.report, ref_km, ascent);
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
            let s = score(&report, ref_km, 0.0);
            Ok(Scored {
                candidate,
                route: r,
                report,
                score: s,
                elevation: None,
                corridor_used_m: params.half_width_m * 2.0,
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

// ---- Closed-loop shape mode (country-search analogue for shapes) -------------

#[derive(Clone, Debug)]
pub struct ShapeCandidate {
    pub center: LatLon,
    pub kind: ShapeKind,
    pub perimeter_km: f64,
    pub rotation_deg: f64,
}

/// A successfully-routed closed-loop shape. Mirrors `Scored` but for shapes:
/// the routed path is one continuous concatenation of legs and the metrics
/// are aggregated across legs (max_deviation isn't meaningful since each leg
/// has its own line).
#[derive(Debug, Clone)]
pub struct ScoredShape {
    pub candidate: ShapeCandidate,
    pub vertices: Vec<LatLon>,
    pub route: RouteResult,
    pub surface_km: [f64; 6],
    pub ferry_km: f64,
    pub score: f64,
    /// Worst single-leg detour ratio above the intended straight shape leg.
    /// `0.0` means every routed leg matched its target length; `1.0` means at
    /// least one leg routed twice as long as intended. This is the key shape
    /// fidelity metric: a single bad leg creates visible tails even when total
    /// loop distance looks acceptable.
    pub max_leg_detour: f64,
    pub elevation: Option<Profile>,
    /// Full-width corridor (metres) actually used to produce this route — see
    /// `Scored::corridor_used_m`.
    pub corridor_used_m: f64,
}

/// Sample N shape candidates whose center sits inside an inset bbox so the
/// shape's outline cannot escape the OSM data. Random center + random
/// rotation; perimeter is fixed (one shape "size" per search).
pub fn generate_shape_candidates_in_bbox(
    bbox: BBox,
    kind: ShapeKind,
    perimeter_km: f64,
    n: usize,
    seed: u64,
) -> Vec<ShapeCandidate> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    let mut rng = StdRng::seed_from_u64(seed);
    let unit_perim = unit_shape_perimeter(kind).max(1e-9);
    let radius_m = perimeter_km * 1000.0 / unit_perim;
    let max_extent_m = radius_m * unit_max_radius(kind);
    // Inset the bbox so any vertex stays inside the data area.
    let mid_lat = (bbox.lat_min + bbox.lat_max) * 0.5;
    let lat_inset_deg = max_extent_m / 111_000.0;
    let lon_inset_deg = max_extent_m / (111_000.0 * mid_lat.to_radians().cos().max(0.1));
    let lat_lo = bbox.lat_min + lat_inset_deg;
    let lat_hi = bbox.lat_max - lat_inset_deg;
    let lon_lo = bbox.lon_min + lon_inset_deg;
    let lon_hi = bbox.lon_max - lon_inset_deg;
    if lat_lo >= lat_hi || lon_lo >= lon_hi {
        // Shape is too large to fit anywhere in the bbox.
        return Vec::new();
    }
    let mut out = Vec::with_capacity(n);
    let lock_rotation = kind.has_natural_orientation();
    for _ in 0..n {
        let center = LatLon::new(rng.gen_range(lat_lo..lat_hi), rng.gen_range(lon_lo..lon_hi));
        let rotation_deg = if lock_rotation {
            0.0
        } else {
            rng.gen_range(0.0..360.0)
        };
        out.push(ShapeCandidate {
            center,
            kind,
            perimeter_km,
            rotation_deg,
        });
    }
    out
}

pub fn route_one_shape(
    graph: &Graph,
    candidate: ShapeCandidate,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
) -> Result<ScoredShape, (ShapeCandidate, String)> {
    route_one_shape_with_limits(graph, candidate, no_go, params, filter, None)
}

pub fn route_one_shape_with_limits(
    graph: &Graph,
    candidate: ShapeCandidate,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
    max_leg_detour_cap: Option<f64>,
) -> Result<ScoredShape, (ShapeCandidate, String)> {
    let vertices = place_shape(
        candidate.center,
        candidate.kind,
        candidate.perimeter_km,
        candidate.rotation_deg,
    );
    if vertices.len() < 2 {
        return Err((candidate, "shape has no legs".into()));
    }
    use petgraph::graph::NodeIndex;
    let mut all_points: Vec<LatLon> = Vec::new();
    let mut all_edges: Vec<EdgeData> = Vec::new();
    let mut all_nodes: Vec<NodeIndex> = Vec::new();
    let mut start_prev: Option<NodeIndex> = None;
    // Track per-leg fidelity: how much each routed leg stretches beyond the
    // straight-line distance between consecutive polyline vertices. A loop
    // with one wildly detouring leg should rank below a loop where every leg
    // hugs the polyline, even if their total lengths match.
    let mut max_leg_detour: f64 = 0.0;
    for (i, w) in vertices.windows(2).enumerate() {
        let pair = match graph.closest_connected_pair(w[0], w[1]) {
            Some(p) => p,
            None => return Err((candidate, format!("leg {}: graph empty", i + 1))),
        };
        let (s_idx, e_idx, sd, ed) = pair;
        if sd > filter.snap_tolerance_m || ed > filter.snap_tolerance_m {
            return Err((
                candidate,
                format!(
                    "leg {}: snap displacement too large ({:.0} m / {:.0} m)",
                    i + 1,
                    sd,
                    ed
                ),
            ));
        }
        let prev_to_pass = match (all_nodes.last(), start_prev) {
            (Some(&last), Some(_)) if last == s_idx => start_prev,
            _ => None,
        };
        // Fall back to unthreaded routing if the U-turn-forbid leaves the
        // start with nowhere to go (degree-1 dead-end snap).
        let leg = match route::shortest_with_start_prev(
            graph,
            s_idx,
            e_idx,
            prev_to_pass,
            w[0],
            w[1],
            params,
        ) {
            Ok(r) => r,
            Err(_) if prev_to_pass.is_some() => {
                match route::shortest_with_start_prev(graph, s_idx, e_idx, None, w[0], w[1], params)
                {
                    Ok(r) => r,
                    Err(e) => return Err((candidate, format!("leg {}: {}", i + 1, e))),
                }
            }
            Err(e) => {
                return Err((candidate, format!("leg {}: {}", i + 1, e)));
            }
        };
        let intended = haversine(w[0], w[1]).max(1.0);
        let leg_detour = (leg.total_length_m / intended - 1.0).max(0.0);
        if leg_detour > max_leg_detour {
            max_leg_detour = leg_detour;
        }
        if let Some(cap) = max_leg_detour_cap {
            if max_leg_detour > cap {
                return Err((
                    candidate,
                    format!(
                        "leg {}: detour {:.0}% exceeds shape fidelity cap {:.0}%",
                        i + 1,
                        leg_detour * 100.0,
                        cap * 100.0
                    ),
                ));
            }
        }
        if all_points.is_empty() {
            all_points.extend(&leg.points);
            all_nodes.extend(&leg.node_indices);
        } else {
            all_points.extend(&leg.points[1..]);
            all_nodes.extend(&leg.node_indices[1..]);
        }
        all_edges.extend(&leg.edges);
        start_prev = if leg.node_indices.len() >= 2 {
            Some(leg.node_indices[leg.node_indices.len() - 2])
        } else {
            None
        };
    }
    // Geometric loop pruning catches the cross-leg "tail" pattern (a lobe
    // that bridges leg boundaries via a parallel road). 30 m radius is
    // tight enough to leave real cross-overs at major junctions untouched.
    let bike_or_mtb = params.modes & (crate::osm::MODE_BIKE | crate::osm::MODE_MTB) != 0;
    if bike_or_mtb {
        route::prune_geometric_loops(&mut all_nodes, &mut all_points, &mut all_edges, 60.0, 200.0);
    }
    let mut surface_km = [0.0_f64; 6];
    let mut ferry_km = 0.0;
    for e in &all_edges {
        let s = (e.surface as usize).min(5);
        surface_km[s] += e.length_m / 1000.0;
        if (e.modes & MODE_FERRY) != 0 {
            ferry_km += e.length_m / 1000.0;
        }
    }
    let total_length: f64 = all_edges.iter().map(|e| e.length_m).sum();
    let real_km = total_length / 1000.0;
    let detour = (real_km / candidate.perimeter_km - 1.0).max(0.0);
    let route = RouteResult {
        points: all_points,
        edges: all_edges,
        total_length_m: total_length,
        node_indices: all_nodes,
    };
    let report = viability::analyse(&route, vertices[0], vertices[1], no_go, f64::INFINITY);
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
    // Combined score: total detour as before, plus a heavy penalty on the
    // worst single leg. A loop where one leg detours 200% but the rest are
    // tight will end up scoring worse than a loop where every leg detours
    // ~50% — exactly what the user wants for shape fidelity.
    let score = 100.0 * detour + 80.0 * max_leg_detour;
    Ok(ScoredShape {
        candidate,
        vertices,
        route,
        surface_km,
        ferry_km,
        score,
        max_leg_detour,
        elevation: None,
        corridor_used_m: params.half_width_m * 2.0,
    })
}

pub fn evaluate_shapes(
    graph: &Graph,
    candidates: Vec<ShapeCandidate>,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
    progress: Option<&AtomicUsize>,
) -> Vec<Result<ScoredShape, (ShapeCandidate, String)>> {
    evaluate_shapes_with_limits(graph, candidates, no_go, params, filter, progress, None)
}

pub fn evaluate_shapes_with_limits(
    graph: &Graph,
    candidates: Vec<ShapeCandidate>,
    no_go: &[NoGoZone],
    params: &RouteParams,
    filter: &RouteFilter,
    progress: Option<&AtomicUsize>,
    max_leg_detour_cap: Option<f64>,
) -> Vec<Result<ScoredShape, (ShapeCandidate, String)>> {
    use rayon::prelude::*;
    candidates
        .into_par_iter()
        .map(|c| {
            let r =
                route_one_shape_with_limits(graph, c, no_go, params, filter, max_leg_detour_cap);
            if let Some(p) = progress {
                p.fetch_add(1, Ordering::Relaxed);
            }
            r
        })
        .collect()
}

/// Recompute a shape's score after elevation is filled in. Detour ratio plus
/// a per-km ascent term so flat loops outrank steeply climbing ones.
pub fn rescore_shape_with_elevation(s: &mut ScoredShape) {
    let real_km = s.route.total_length_m / 1000.0;
    let detour = if s.candidate.perimeter_km > 0.0 {
        (real_km / s.candidate.perimeter_km - 1.0).max(0.0)
    } else {
        0.0
    };
    let ascent = s.elevation.as_ref().map(|e| e.ascent_m).unwrap_or(0.0);
    let per_km = if real_km > 0.0 { ascent / real_km } else { 0.0 };
    s.score = 100.0 * detour + 80.0 * s.max_leg_detour + 0.05 * per_km;
}

/// Rank by score, drop near-duplicate centers, take top n. Two shapes are
/// duplicates if their centers sit within `min_sep_km`.
pub fn rank_dedup_top_shapes(
    mut scored: Vec<ScoredShape>,
    n: usize,
    min_sep_km: f64,
) -> Vec<ScoredShape> {
    scored.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());
    let mut out: Vec<ScoredShape> = Vec::with_capacity(n);
    let sep_m = min_sep_km * 1000.0;
    for s in scored {
        if out.len() >= n {
            break;
        }
        let dup = out
            .iter()
            .any(|kept| haversine(kept.candidate.center, s.candidate.center) < sep_m);
        if !dup {
            out.push(s);
        }
    }
    out
}

/// Sort the surviving Scored entries by ascending score (easiest first).
pub fn rank(mut scored: Vec<Scored>) -> Vec<Scored> {
    scored.sort_by(|a, b| a.score.partial_cmp(&b.score).unwrap());
    scored
}

/// Rank, deduplicate near-identical pairs, take top `n`. Two candidates count
/// as duplicates if both their start AND end points are within `min_sep_km`.
pub fn rank_dedup_top(mut scored: Vec<Scored>, n: usize, min_sep_km: f64) -> Vec<Scored> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::{NodePoint, BCLASS_RESIDENTIAL, MODE_FOOT, SURFACE_PAVED};
    use geo::{Coord, LineString, MultiPolygon, Polygon};
    use petgraph::graph::UnGraph;
    use petgraph::visit::EdgeRef;
    use rustc_hash::FxHashMap;

    fn graph_for_shape(vertices: &[LatLon]) -> Graph {
        let mut g: UnGraph<LatLon, EdgeData> = UnGraph::new_undirected();
        let nodes: Vec<_> = vertices[..vertices.len() - 1]
            .iter()
            .map(|p| g.add_node(*p))
            .collect();
        for i in 0..nodes.len() {
            let a = nodes[i];
            let b = nodes[(i + 1) % nodes.len()];
            g.add_edge(
                a,
                b,
                EdgeData {
                    length_m: haversine(g[a], g[b]),
                    modes: MODE_FOOT,
                    surface: SURFACE_PAVED,
                    bike_attrs: BCLASS_RESIDENTIAL,
                    way_id: 1,
                },
            );
        }
        let pts: Vec<NodePoint> = g
            .node_indices()
            .map(|ni| NodePoint {
                idx: ni,
                lat: g[ni].lat,
                lon: g[ni].lon,
            })
            .collect();
        let mut uf = petgraph::unionfind::UnionFind::new(g.node_count());
        for er in g.edge_references() {
            uf.union(er.source().index(), er.target().index());
        }
        let components: Vec<u32> = (0..g.node_count()).map(|i| uf.find(i) as u32).collect();
        Graph {
            graph: g,
            osm_to_idx: FxHashMap::default(),
            components,
            rtree: rstar::RTree::bulk_load(pts),
            elevations_m: None,
        }
    }

    fn no_go_around_segment(name: &str, a: LatLon, b: LatLon) -> NoGoZone {
        let pad = 0.0001;
        let lon_min = a.lon.min(b.lon) - pad;
        let lon_max = a.lon.max(b.lon) + pad;
        let lat_min = a.lat.min(b.lat) - pad;
        let lat_max = a.lat.max(b.lat) + pad;
        NoGoZone {
            name: name.to_string(),
            polygons: MultiPolygon(vec![Polygon::new(
                LineString::from(vec![
                    Coord {
                        x: lon_min,
                        y: lat_min,
                    },
                    Coord {
                        x: lon_max,
                        y: lat_min,
                    },
                    Coord {
                        x: lon_max,
                        y: lat_max,
                    },
                    Coord {
                        x: lon_min,
                        y: lat_max,
                    },
                    Coord {
                        x: lon_min,
                        y: lat_min,
                    },
                ]),
                vec![],
            )]),
        }
    }

    #[test]
    fn shape_route_rejects_no_go_intersection() {
        let candidate = ShapeCandidate {
            center: LatLon::new(0.0, 0.0),
            kind: ShapeKind::Square,
            perimeter_km: 4.0,
            rotation_deg: 0.0,
        };
        let vertices = place_shape(
            candidate.center,
            candidate.kind,
            candidate.perimeter_km,
            candidate.rotation_deg,
        );
        let graph = graph_for_shape(&vertices);
        let params = RouteParams {
            modes: MODE_FOOT,
            half_width_m: 200.0,
            alpha: 4.0,
            corridor_max_m: 200.0,
            paved_only: false,
        };
        let filter = RouteFilter::lax(200.0);
        let no_go = vec![no_go_around_segment("blocked", vertices[0], vertices[1])];

        let err = route_one_shape(&graph, candidate, &no_go, &params, &filter)
            .expect_err("shape route should be rejected by no-go zone")
            .1;

        assert!(err.contains("no-go: blocked"), "unexpected error: {err}");
    }

    #[test]
    fn shape_elevation_rescore_keeps_worst_leg_penalty() {
        let candidate = ShapeCandidate {
            center: LatLon::new(0.0, 0.0),
            kind: ShapeKind::Square,
            perimeter_km: 10.0,
            rotation_deg: 0.0,
        };
        let mut scored = ScoredShape {
            candidate,
            vertices: vec![LatLon::new(0.0, 0.0), LatLon::new(0.0, 0.01)],
            route: RouteResult {
                points: vec![LatLon::new(0.0, 0.0), LatLon::new(0.0, 0.01)],
                edges: Vec::new(),
                total_length_m: 12_000.0,
                node_indices: Vec::new(),
            },
            surface_km: [0.0; 6],
            ferry_km: 0.0,
            score: 0.0,
            max_leg_detour: 1.5,
            elevation: None,
            corridor_used_m: 1_000.0,
        };

        rescore_shape_with_elevation(&mut scored);

        assert!(
            scored.score >= 140.0,
            "score should retain large worst-leg penalty, got {}",
            scored.score
        );
    }

    #[test]
    fn shape_route_can_abort_on_bad_leg_detour() {
        let candidate = ShapeCandidate {
            center: LatLon::new(0.0, 0.0),
            kind: ShapeKind::Square,
            perimeter_km: 4.0,
            rotation_deg: 0.0,
        };
        let vertices = place_shape(
            candidate.center,
            candidate.kind,
            candidate.perimeter_km,
            candidate.rotation_deg,
        );
        let graph = graph_for_shape(&vertices);
        let params = RouteParams {
            modes: MODE_FOOT,
            half_width_m: 200.0,
            alpha: 4.0,
            corridor_max_m: 200.0,
            paved_only: false,
        };
        let filter = RouteFilter::lax(200.0);

        let err = route_one_shape_with_limits(&graph, candidate, &[], &params, &filter, Some(0.0))
            .expect_err("any nonzero detour should trip zero cap")
            .1;

        assert!(
            err.contains("shape fidelity cap"),
            "unexpected error: {err}"
        );
    }
}
