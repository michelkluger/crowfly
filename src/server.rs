//! Local HTTP server that holds the routing graph in memory and serves an
//! interactive frontend. Two search modes:
//!
//! - "between": user clicks two points, server returns alternative routes
//!   with varying corridor strictness (alpha sweep).
//! - "country": server samples diverse start/end pairs across the bbox,
//!   routes them, dedups, returns the top N.
//!
//! The server is single-process, single-graph, single-user — meant to run
//! on `localhost` while you plan trips.

use crate::dem::Dem;
use crate::elevation::profile_from_dem;
use crate::explore::{self, Candidate, RouteFilter, Scored, ScoredShape};
use crate::geodesy::{corridor_polygon, haversine, LatLon};
use crate::hershey_fonts::Font;
use crate::osm::{self, BBox, Graph, SURFACE_LABELS};
use crate::route::RouteParams;
use crate::text;
use crate::viability::NoGoZone;
use anyhow::{anyhow, Result};
use axum::{
    extract::{DefaultBodyLimit, State},
    http::{header, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use rayon::prelude::*;
use serde::Deserialize;
use serde_json::{json, Value};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;

pub struct ServerState {
    pub graph: Graph,
    pub bbox: BBox,
    pub no_go: Vec<NoGoZone>,
    pub long_edge_threshold_m: f64,
    /// DEM kept resident after stamping node elevations, so route elevation
    /// profiles are sampled locally (no network, no rate limit). `None` when
    /// DEM loading is disabled; responses then carry no elevation.
    pub dem: Option<Dem>,
    /// Latest in-flight search progress. Cleared when no work is running. The
    /// server is single-user (README), so a single slot is enough.
    pub progress: Mutex<Option<ProgressTracker>>,
    /// Cancel flag of the most recent search; see [`Search`].
    pub search_cancel: Mutex<Arc<AtomicBool>>,
    /// The country's edge — outermost road node per bearing from the centre
    /// — where crossing ("border to border") searches start and end.
    pub outline: Vec<explore::OutlinePoint>,
}

impl ServerState {
    /// State for serving `graph`: derives its bbox and outline.
    pub fn new(
        graph: Graph,
        no_go: Vec<NoGoZone>,
        long_edge_threshold_m: f64,
        dem: Option<Dem>,
    ) -> Self {
        let bbox = graph.bbox();
        let outline = explore::country_outline(&graph, 360);
        Self {
            graph,
            bbox,
            no_go,
            long_edge_threshold_m,
            dem,
            progress: Mutex::new(None),
            search_cancel: Default::default(),
            outline,
        }
    }
}

/// One search's cancel flag and its claim on the shared progress slot.
/// Starting a search cancels the previous one: the server is single-user,
/// and the FE has already abandoned the old request when it starts a new
/// one. A superseded search may take a moment to wind down, so it only
/// touches the progress slot while it still owns it.
#[derive(Clone)]
struct Search {
    state: Arc<ServerState>,
    cancel: Arc<AtomicBool>,
    done: Arc<AtomicUsize>,
}

/// The handler's owning handle on its [`Search`]. Dropping it — when the
/// handler returns, or when axum drops the handler because the client went
/// away (an aborted fetch) — cancels any remaining work and releases the
/// progress slot, so neither outlives the request.
struct SearchGuard(Search);

impl Search {
    fn begin(state: &Arc<ServerState>, kind: &'static str, total: usize) -> SearchGuard {
        let cancel = Arc::new(AtomicBool::new(false));
        std::mem::replace(&mut *state.search_cancel.lock().unwrap(), cancel.clone())
            .store(true, Ordering::Relaxed);
        let done = Arc::new(AtomicUsize::new(0));
        *state.progress.lock().unwrap() = Some(ProgressTracker {
            kind,
            total,
            done: done.clone(),
            started_ms: now_ms(),
        });
        SearchGuard(Self {
            state: state.clone(),
            cancel,
            done,
        })
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }

    /// Grow the progress bar's denominator, e.g. when a retry batch starts.
    fn add_total(&self, n: usize) {
        let mut p = self.state.progress.lock().unwrap();
        if let Some(pt) = p.as_mut().filter(|pt| Arc::ptr_eq(&pt.done, &self.done)) {
            pt.total += n;
        }
    }
}

impl SearchGuard {
    /// Handle for the blocking task doing the work.
    fn task(&self) -> Search {
        self.0.clone()
    }

    fn cancelled(&self) -> bool {
        self.0.cancelled()
    }
}

impl Drop for SearchGuard {
    fn drop(&mut self) {
        self.0.cancel.store(true, Ordering::Relaxed);
        let mut p = self.0.state.progress.lock().unwrap();
        if p.as_ref()
            .is_some_and(|pt| Arc::ptr_eq(&pt.done, &self.0.done))
        {
            *p = None;
        }
    }
}

/// Counters bucketing why candidates were rejected during a search. Returned
/// alongside the route list so the empty-state UI can tell the user *which*
/// knob to relax (corridor too tight, endpoints unreachable, etc.) instead of
/// a generic "no viable routes" message.
#[derive(Default, Clone, Debug)]
pub struct RejectionStats {
    pub snap_too_far: usize,
    pub corridor_violated: usize,
    pub no_route: usize,
    pub no_go: usize,
    pub too_short: usize,
    pub too_long: usize,
    /// Shape candidates with a leg that detoured past the fidelity cap.
    pub too_winding: usize,
    pub other: usize,
}

impl RejectionStats {
    pub fn classify(&mut self, msg: &str) {
        let m = msg.to_ascii_lowercase();
        if m.contains("endpoint too far from graph") || m.contains("snap displacement") {
            self.snap_too_far += 1;
        } else if m.contains("exceeds corridor half-width") {
            self.corridor_violated += 1;
        } else if m.starts_with("no-go") || m.contains(": no-go") {
            self.no_go += 1;
        } else if m.contains("below minimum") {
            self.too_short += 1;
        } else if m.contains("above maximum") {
            self.too_long += 1;
        } else if m.contains("no route") || m.contains("graph empty") {
            self.no_route += 1;
        } else if m.contains("shape fidelity cap") {
            self.too_winding += 1;
        } else {
            self.other += 1;
        }
    }
    pub fn merge(&mut self, other: &RejectionStats) {
        self.snap_too_far += other.snap_too_far;
        self.corridor_violated += other.corridor_violated;
        self.no_route += other.no_route;
        self.no_go += other.no_go;
        self.too_short += other.too_short;
        self.too_long += other.too_long;
        self.too_winding += other.too_winding;
        self.other += other.other;
    }
    pub fn total(&self) -> usize {
        self.snap_too_far
            + self.corridor_violated
            + self.no_route
            + self.no_go
            + self.too_short
            + self.too_long
            + self.too_winding
            + self.other
    }
    pub fn to_json(&self) -> Value {
        json!({
            "snap_too_far":      self.snap_too_far,
            "corridor_violated": self.corridor_violated,
            "no_route":          self.no_route,
            "no_go":             self.no_go,
            "too_short":         self.too_short,
            "too_long":          self.too_long,
            "too_winding":       self.too_winding,
            "other":             self.other,
            "total":             self.total(),
        })
    }
}

/// Retry schedule steps: (candidate sample factor, full-corridor widening
/// km). Country search grows its oversample late so the first attempts stay
/// snappy, and widens by an absolute delta so the worst case is request
/// +2 km — never a multiplicative blow-up. Shape search widens from batch 2.
const COUNTRY_SCHEDULE: [(f64, f64); 8] = [
    (1.0, 0.0),
    (1.0, 0.0),
    (1.5, 0.0),
    (1.5, 0.5),
    (2.0, 1.0),
    (2.0, 1.5),
    (2.5, 2.0),
    (2.5, 2.0),
];
const SHAPE_SCHEDULE: [(f64, f64); 5] =
    [(1.0, 0.0), (1.5, 0.5), (2.0, 1.0), (2.5, 1.5), (3.0, 2.0)];

/// Inputs of the persistence loop shared by country and shape search.
struct Retry<'a> {
    schedule: &'a [(f64, f64)],
    /// Never widen the corridor.
    strict: bool,
    base_half_m: f64,
    base_sample: usize,
    seed: u64,
    min_viable: usize,
}

/// One batch of [`run_batches`].
struct Batch {
    /// Corridor half-width: the request's plus half the widening.
    half_width_m: f64,
    /// Full-corridor widening applied, km (0 when strict).
    widen_km: f64,
    n_sample: usize,
    /// Per-batch seed, so a retry doesn't redraw candidates it already
    /// proved infeasible.
    seed: u64,
}

/// Run `retry`'s batches until `min_viable` results are found, the schedule
/// runs out, the search is cancelled, or `route` returns None (nothing left
/// to sample). Rejections are bucketed for the empty-state UI.
fn run_batches<T, C>(
    task: &Search,
    label: &str,
    retry: &Retry,
    mut route: impl FnMut(&Batch) -> Option<Vec<Result<T, (C, String)>>>,
) -> (Vec<T>, RejectionStats) {
    let mut all = Vec::new();
    let mut stats = RejectionStats::default();
    for (i, &(sample_factor, widen_km)) in retry.schedule.iter().enumerate() {
        if task.cancelled() {
            break;
        }
        let started = std::time::Instant::now();
        let widen_km = if retry.strict { 0.0 } else { widen_km };
        let batch = Batch {
            half_width_m: retry.base_half_m + widen_km * 500.0,
            widen_km,
            n_sample: ((retry.base_sample as f64) * sample_factor).round() as usize,
            seed: retry
                .seed
                .wrapping_add((i as u64).wrapping_mul(0x9E3779B97F4A7C15)),
        };
        let Some(results) = route(&batch) else {
            break;
        };
        for r in results {
            match r {
                Ok(s) => all.push(s),
                Err((_, msg)) => stats.classify(&msg),
            }
        }
        eprintln!(
            "{label} batch {}: {} candidates, {} viable total, {:.1}s",
            i + 1,
            batch.n_sample,
            all.len(),
            started.elapsed().as_secs_f64()
        );
        if all.len() >= retry.min_viable {
            break;
        }
    }
    (all, stats)
}

/// In-flight search progress. `done` is mutated lock-free from the rayon worker
/// pool; the HTTP read-side reads the atomic.
pub struct ProgressTracker {
    pub kind: &'static str,
    pub total: usize,
    pub done: Arc<AtomicUsize>,
    pub started_ms: u128,
}

#[derive(Clone)]
pub struct AppState(pub Arc<ServerState>);

#[derive(Deserialize)]
pub struct BetweenReq {
    pub start: [f64; 2],
    pub end: [f64; 2],
    #[serde(default = "default_modes")]
    pub modes: String,
    #[serde(default = "default_width")]
    pub width_m: f64,
    /// Number of alternatives — varied by alpha (corridor strictness).
    #[serde(default = "default_alts")]
    pub alternatives: usize,
    #[serde(default = "default_elev_samples")]
    pub elevation_samples: usize,
    #[serde(default)]
    pub paved_only: bool,
}

#[derive(Deserialize)]
pub struct CountryReq {
    #[serde(default = "default_modes")]
    pub modes: String,
    #[serde(default = "default_width")]
    pub width_m: f64,
    #[serde(default = "default_count")]
    pub count: usize,
    /// Straight-line length bounds (km). Both the line and its routed
    /// distance must fall inside. Optional for crossings, which are bounded
    /// by the country itself; other lines default to 100–220 km.
    #[serde(default)]
    pub distance_min_km: Option<f64>,
    #[serde(default)]
    pub distance_max_km: Option<f64>,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default = "default_elev_samples")]
    pub elevation_samples: usize,
    /// Only lines that cross the country: both ends on its outline (the
    /// outermost roads, i.e. at the border), on roughly opposite sides.
    #[serde(default)]
    pub border_to_border: bool,
    /// Keep generating fresh batches of candidates until at least this many
    /// viable routes are found, up to `max_batches` attempts. Defaults to
    /// `count` (try to fill the requested number).
    #[serde(default)]
    pub min_viable: Option<usize>,
    #[serde(default)]
    pub paved_only: bool,
    /// When true, every retry batch uses the exact requested corridor — no
    /// widening to recover from empty batches. Search may end with zero
    /// results if the requested corridor is infeasible.
    #[serde(default)]
    pub strict_corridor: bool,
}

#[derive(Deserialize)]
pub struct ShapeSearchReq {
    /// Shape identifier (`circle`, `triangle`, `square`, `pentagon`, `hexagon`,
    /// `star`, `heart`, `figure-8`, `swiss-cross`).
    pub kind: String,
    /// Loop length in km — every candidate is sized to this.
    pub perimeter_km: f64,
    #[serde(default = "default_modes")]
    pub modes: String,
    #[serde(default = "default_width")]
    pub width_m: f64,
    #[serde(default = "default_count")]
    pub count: usize,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default)]
    pub paved_only: bool,
    #[serde(default = "default_elev_samples")]
    pub elevation_samples: usize,
    #[serde(default)]
    pub min_viable: Option<usize>,
    #[serde(default)]
    pub strict_corridor: bool,
}

#[derive(Deserialize)]
pub struct TextReq {
    /// What to write. Newlines force line breaks; non-ASCII glyphs are
    /// dropped (counted in `missing_glyphs`).
    pub text: String,
    /// Capital-letter height in metres. Below ~5 km the routed letters
    /// generally lose legibility because the road grid spacing dominates
    /// stroke length; default is 15 km.
    #[serde(default = "default_text_letter")]
    pub letter_height_m: f64,
    /// Wrap to a new line when a line's horizontal extent would exceed this
    /// many metres. 0 disables wrapping; omitted = 80 % of the region's
    /// east–west width, so the message fits the country.
    #[serde(default)]
    pub wrap_m: Option<f64>,
    /// Font name: `simplex` or `cursive`.
    #[serde(default = "default_text_font")]
    pub font: String,
    /// Per-stroke detour cap. A placement is accepted only if every stroke
    /// routes within `intended × this`. Lower = stricter shape fidelity but
    /// more candidate placements rejected. 1.6 means up to +60 % detour.
    #[serde(default = "default_text_detour_cap")]
    pub detour_cap: f64,
    /// Number of (centre, bearing) candidates the search tries before giving
    /// up. Each candidate roughly costs `~stroke_count` short A* routes;
    /// mountain candidates abort on the first stroke. 200 fans well across a
    /// country-sized graph.
    #[serde(default = "default_text_candidates")]
    pub candidates: usize,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default = "default_modes")]
    pub modes: String,
    #[serde(default)]
    pub paved_only: bool,
    #[serde(default = "default_elev_samples")]
    pub elevation_samples: usize,
}

fn default_text_letter() -> f64 {
    15_000.0
}
fn default_text_font() -> String {
    "simplex".into()
}
fn default_text_detour_cap() -> f64 {
    1.6
}
fn default_text_candidates() -> usize {
    200
}

fn default_modes() -> String {
    "foot,bike".into()
}
fn default_width() -> f64 {
    8000.0
}
fn default_alts() -> usize {
    3
}
fn default_count() -> usize {
    8
}
fn default_dist_min() -> f64 {
    100.0
}
fn default_dist_max() -> f64 {
    220.0
}
fn default_elev_samples() -> usize {
    30
}

/// `[{label, km}]` for every surface category the route uses.
fn surface_json(surface_km: &[f64; 6]) -> Vec<Value> {
    SURFACE_LABELS
        .iter()
        .zip(surface_km)
        .filter(|(_, &km)| km > 0.001)
        .map(|(label, km)| json!({ "label": label, "km": km }))
        .collect()
}

fn scored_to_json(s: &Scored, rank: usize, width_m: f64) -> Value {
    let r = &s.report;
    let coords: Vec<[f64; 2]> = s.route.points.iter().map(|p| [p.lon, p.lat]).collect();
    let corridor: Vec<[f64; 2]> = corridor_polygon(s.candidate.start, s.candidate.end, width_m)
        .into_iter()
        .map(|p| [p.lon, p.lat])
        .collect();
    let ref_km = haversine(s.candidate.start, s.candidate.end) / 1000.0;
    json!({
        "rank": rank,
        "score": s.score,
        "start": [s.candidate.start.lon, s.candidate.start.lat],
        "end":   [s.candidate.end.lon,   s.candidate.end.lat],
        "bearing_deg": s.candidate.bearing_deg,
        "ref_km": ref_km,
        "real_km": r.total_km,
        "max_dev_m": r.max_deviation_m,
        "ferry_km": r.ferry_km,
        "surface": surface_json(&r.surface_km),
        "elevation": s.elevation,
        "coords": coords,
        "corridor": corridor,
        "width_m": width_m,
        "corridor_used_m": s.corridor_used_m,
    })
}

async fn handle_between(
    State(state): State<AppState>,
    Json(req): Json<BetweenReq>,
) -> Result<Json<Value>, AppError> {
    let modes = osm::parse_modes(&req.modes).map_err(AppError::bad)?;
    let start = LatLon::new(req.start[0], req.start[1]);
    let end = LatLon::new(req.end[0], req.end[1]);
    let half = req.width_m / 2.0;

    // Vary alpha to get qualitatively different routes.
    let alphas = pick_alphas(req.alternatives.max(1));
    let inner = state.0.clone();

    let search = Search::begin(&inner, "between", alphas.len());
    let task = search.task();
    let paved_only = req.paved_only;
    let elevation_samples = req.elevation_samples;
    let scored_result = tokio::task::spawn_blocking(move || -> (Vec<Scored>, Vec<String>) {
        let mut out = Vec::with_capacity(alphas.len());
        let mut errors: Vec<String> = Vec::new();
        let filter = RouteFilter::lax(half.max(2_000.0));
        for a in alphas {
            if task.cancelled() {
                break;
            }
            let params = RouteParams {
                modes,
                half_width_m: half,
                alpha: a,
                corridor_max_m: half,
                paved_only,
            };
            let cand = Candidate::from_endpoints(start, end);
            match explore::route_one(
                &inner.graph,
                cand,
                &inner.no_go,
                &params,
                &filter,
                inner.long_edge_threshold_m,
            ) {
                Ok(s) => out.push(s),
                Err((_, msg)) => errors.push(format!("α={a}: {msg}")),
            }
            task.done.fetch_add(1, Ordering::Relaxed);
        }
        enrich_scored(inner.dem.as_ref(), &mut out, elevation_samples);
        (out, errors)
    })
    .await;
    let (scored, errors) = scored_result.map_err(AppError::internal)?;
    if search.cancelled() {
        return Err(AppError::cancelled());
    }

    if scored.is_empty() {
        let detail = if errors.is_empty() {
            String::new()
        } else {
            format!(" ({})", errors.join("; "))
        };
        return Err(AppError::bad(anyhow!(
            "no route found between those points (try widening corridor or adding modes){}",
            detail
        )));
    }

    let deduped = explore::rank_dedup_top(scored, req.alternatives, 0.5);

    let width_m = req.width_m;
    let resp: Vec<Value> = deduped
        .iter()
        .enumerate()
        .map(|(i, s)| scored_to_json(s, i + 1, width_m))
        .collect();
    Ok(Json(json!({ "results": resp })))
}

async fn handle_country(
    State(state): State<AppState>,
    Json(req): Json<CountryReq>,
) -> Result<Json<Value>, AppError> {
    let modes = osm::parse_modes(&req.modes).map_err(AppError::bad)?;
    let base_half = req.width_m / 2.0;
    let count = req.count.clamp(1, 50);
    let base_sample = (count * 6).max(20); // oversample, then dedupe to top N
    let seed = seed_or_random(req.seed);
    let inner = state.0.clone();
    let bbox = inner.bbox;
    let border = req.border_to_border;
    // Crossings are bounded by the country itself; other lines need a range.
    let (dmin, dmax) = if border {
        (req.distance_min_km, req.distance_max_km)
    } else {
        (
            Some(req.distance_min_km.unwrap_or_else(default_dist_min)),
            Some(req.distance_max_km.unwrap_or_else(default_dist_max)),
        )
    };
    let long_edge = inner.long_edge_threshold_m;
    let min_viable = req.min_viable.unwrap_or(count).max(1);
    let strict = req.strict_corridor;

    let search = Search::begin(&inner, "country", 0);
    let task = search.task();
    let inner_for_task = inner.clone();
    let paved_only = req.paved_only;
    let elevation_samples = req.elevation_samples;
    let scored_result = tokio::task::spawn_blocking(move || -> (Vec<Scored>, RejectionStats) {
        let retry = Retry {
            schedule: &COUNTRY_SCHEDULE,
            strict,
            base_half_m: base_half,
            base_sample,
            seed,
            min_viable,
        };
        let (mut all, stats) = run_batches(&task, "country", &retry, |b| {
            let params = RouteParams {
                modes,
                half_width_m: b.half_width_m,
                alpha: 4.0,
                corridor_max_m: b.half_width_m,
                paved_only,
            };
            let filter = RouteFilter {
                distance_min_km: dmin,
                distance_max_km: dmax,
                snap_tolerance_m: b.half_width_m.max(2_000.0),
                enforce_corridor: true,
            };
            let (lo, hi) = (dmin.unwrap_or(0.0), dmax.unwrap_or(f64::INFINITY));
            let candidates = if border {
                let outline = &inner_for_task.outline;
                explore::generate_crossing_candidates(outline, b.n_sample, lo, hi, b.seed)
            } else {
                explore::generate_country_candidates_in_bbox(bbox, b.n_sample, lo, hi, b.seed)
            };
            task.add_total(candidates.len());
            Some(explore::evaluate(
                &inner_for_task.graph,
                candidates,
                &inner_for_task.no_go,
                &params,
                &filter,
                long_edge,
                Some(&task.done),
                Some(&task.cancel),
            ))
        });
        // Elevation is local and cheap, so profile every viable route and
        // let the ascent term take part in the ranking below.
        enrich_scored(inner_for_task.dem.as_ref(), &mut all, elevation_samples);
        all.iter_mut().for_each(explore::rescore_with_elevation);
        (all, stats)
    })
    .await;
    let (scored, stats) = scored_result.map_err(AppError::internal)?;
    if search.cancelled() {
        return Err(AppError::cancelled());
    }

    // Per-section dedup: routes inside the requested corridor first, then
    // the ones the retry loop found by widening it.
    let width_m = req.width_m;
    let (tight, loose): (Vec<Scored>, Vec<Scored>) = scored
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let mut ranked = explore::rank_dedup_top(tight, count, 8.0);
    ranked.extend(explore::rank_dedup_top(loose, count, 8.0));
    let resp: Vec<Value> = ranked
        .iter()
        .enumerate()
        .map(|(i, s)| scored_to_json(s, i + 1, width_m))
        .collect();
    Ok(Json(json!({
        "seed": seed,
        "results": resp,
        "failures": stats.to_json(),
    })))
}

fn shape_to_json(s: &ScoredShape, rank: usize, width_m: f64) -> Value {
    let coords: Vec<[f64; 2]> = s.route.points.iter().map(|p| [p.lon, p.lat]).collect();
    let vertex_coords: Vec<[f64; 2]> = s.vertices.iter().map(|v| [v.lon, v.lat]).collect();
    let real_km = s.route.total_length_m / 1000.0;
    json!({
        "rank": rank,
        "score": s.score,
        // Use the center as both start and end so the existing FE marker logic
        // (which renders points labelled "start"/"end") drops a single dot at
        // the loop's center.
        "start": [s.candidate.center.lon, s.candidate.center.lat],
        "end":   [s.candidate.center.lon, s.candidate.center.lat],
        "bearing_deg": s.candidate.rotation_deg,
        "ref_km": s.candidate.perimeter_km,
        "real_km": real_km,
        "max_dev_m": 0.0,
        "ferry_km": s.ferry_km,
        "surface": surface_json(&s.surface_km),
        "elevation": s.elevation,
        "coords": coords,
        "corridor": Vec::<[f64; 2]>::new(),
        "width_m": width_m,
        "corridor_used_m": s.corridor_used_m,
        "shape": {
            "kind": format!("{:?}", s.candidate.kind).to_lowercase(),
            "vertices": vertex_coords,
            "rotation_deg": s.candidate.rotation_deg,
            "perimeter_km": s.candidate.perimeter_km,
            "max_leg_detour_pct": (s.max_leg_detour * 100.0).round(),
            "leg_count": s.vertices.len().saturating_sub(1),
        }
    })
}

async fn handle_shape_search(
    State(state): State<AppState>,
    Json(req): Json<ShapeSearchReq>,
) -> Result<Json<Value>, AppError> {
    use crate::shape::ShapeKind;
    let modes = osm::parse_modes(&req.modes).map_err(AppError::bad)?;
    let kind = ShapeKind::parse(&req.kind)
        .ok_or_else(|| AppError::bad(anyhow!("unknown shape: {}", req.kind)))?;
    if !req.perimeter_km.is_finite() || req.perimeter_km <= 0.0 {
        return Err(AppError::bad(anyhow!(
            "perimeter_km must be > 0 (got {})",
            req.perimeter_km
        )));
    }
    let base_half = req.width_m / 2.0;
    let count = req.count.clamp(1, 50);
    let shape_leg_count =
        crate::shape::place_shape(LatLon::new(0.0, 0.0), kind, req.perimeter_km, 0.0)
            .len()
            .saturating_sub(1)
            .max(1);
    // Shape cost is roughly `candidates × legs × A*`. Hearts/figure-8s have
    // many more legs than polygons, so use fewer candidates there; otherwise a
    // 400 km heart can mean thousands of A* calls before the first response.
    let sample_factor = if shape_leg_count >= 24 {
        3
    } else if shape_leg_count >= 12 {
        4
    } else {
        8
    };
    let base_sample = (count * sample_factor).max(count + 8).max(16);
    let seed = seed_or_random(req.seed);
    let min_viable = req.min_viable.unwrap_or(count).max(1);
    let strict = req.strict_corridor;

    let inner = state.0.clone();
    let search = Search::begin(&inner, "shape-search", 0);
    let task = search.task();
    let inner_for_task = inner.clone();
    let bbox = inner.bbox;
    let perimeter_km = req.perimeter_km;
    let paved_only = req.paved_only;
    let elevation_samples = req.elevation_samples;
    let routed = tokio::task::spawn_blocking(move || -> (Vec<ScoredShape>, RejectionStats) {
        let retry = Retry {
            schedule: &SHAPE_SCHEDULE,
            strict,
            base_half_m: base_half,
            base_sample,
            seed,
            min_viable,
        };
        let (mut all, stats) = run_batches(&task, "shape", &retry, |b| {
            let params = RouteParams {
                modes,
                half_width_m: b.half_width_m,
                alpha: 4.0,
                corridor_max_m: b.half_width_m,
                paved_only,
            };
            let filter = RouteFilter {
                distance_min_km: None,
                distance_max_km: None,
                snap_tolerance_m: b.half_width_m.max(2_000.0),
                enforce_corridor: true,
            };
            // Abort bad shape candidates early. A single leg >120% longer
            // than intended creates the visible tails/kinks we don't want, and
            // continuing to route the remaining 20-30 legs is wasted work.
            let max_leg_detour_cap = Some(1.2 + b.widen_km * 0.15);
            let candidates = explore::generate_shape_candidates_in_bbox(
                bbox,
                kind,
                perimeter_km,
                b.n_sample,
                b.seed,
            );
            if candidates.is_empty() {
                return None; // shape doesn't fit in bbox
            }
            task.add_total(candidates.len());
            Some(explore::evaluate_shapes_with_limits(
                &inner_for_task.graph,
                candidates,
                &inner_for_task.no_go,
                &params,
                &filter,
                Some(&task.done),
                Some(&task.cancel),
                max_leg_detour_cap,
            ))
        });
        // Profile every viable loop so the ascent re-score can promote a
        // flatter loop over a marginally tighter but climbier one.
        enrich_shapes(inner_for_task.dem.as_ref(), &mut all, elevation_samples);
        all.iter_mut()
            .for_each(explore::rescore_shape_with_elevation);
        (all, stats)
    })
    .await;
    let (scored, stats) = routed.map_err(AppError::internal)?;
    if search.cancelled() {
        return Err(AppError::cancelled());
    }

    // Dedup by center proximity, scaled to the shape's diameter so two big
    // shapes don't merge unless they really overlap.
    let dedup_sep_km = (req.perimeter_km / std::f64::consts::TAU).max(8.0);
    let width_m = req.width_m;
    let (tight, loose): (Vec<ScoredShape>, Vec<ScoredShape>) = scored
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let mut ranked = explore::rank_dedup_top_shapes(tight, count, dedup_sep_km);
    ranked.extend(explore::rank_dedup_top_shapes(loose, count, dedup_sep_km));
    let resp: Vec<Value> = ranked
        .iter()
        .enumerate()
        .map(|(i, s)| shape_to_json(s, i + 1, width_m))
        .collect();
    Ok(Json(json!({
        "seed": seed,
        "results": resp,
        "failures": stats.to_json(),
    })))
}

/// Search across candidate placements (centre × bearing) inside the loaded
/// graph's bbox and return the placement where every stroke routed within
/// the detour cap. The user explicitly does NOT want "skipping" semantics:
/// a placement is either accepted whole or discarded — partial successes
/// produce gappy, unreadable letters.
async fn handle_text(
    State(state): State<AppState>,
    Json(req): Json<TextReq>,
) -> Result<Json<Value>, AppError> {
    let modes = osm::parse_modes(&req.modes).map_err(AppError::bad)?;
    if req.text.trim().is_empty() {
        return Err(AppError::bad(anyhow!("text is empty")));
    }
    if !req.letter_height_m.is_finite() || req.letter_height_m < 500.0 {
        return Err(AppError::bad(anyhow!(
            "letter_height_m must be ≥ 500 (got {})",
            req.letter_height_m
        )));
    }
    let font = Font::parse(&req.font)
        .ok_or_else(|| AppError::bad(anyhow!("unknown font: {}", req.font)))?;

    // Stroke-tight corridor: ≈ 10 % of letter height, clamped both sides so
    // small letters still have *some* leeway and giant letters don't get a
    // ribbon so wide that letters read as smudges. Higher α than other modes
    // because letters need shape fidelity, not road preference.
    let stroke_half_m = (0.10 * req.letter_height_m).clamp(300.0, 1_500.0);
    let params = RouteParams {
        modes,
        half_width_m: stroke_half_m,
        alpha: 30.0,
        corridor_max_m: stroke_half_m,
        paved_only: req.paved_only,
    };
    let detour_cap = req.detour_cap.max(1.0);
    let n_candidates = req.candidates.clamp(20, 1000);
    let seed = seed_or_random(req.seed);
    let bbox = state.0.bbox;
    // Default: wrap at 80 % of the region's east–west width, so the message
    // fits the country instead of running off it.
    let wrap_m = req
        .wrap_m
        .unwrap_or_else(|| {
            let mid_lat = (bbox.lat_min + bbox.lat_max) * 0.5;
            let west = LatLon::new(mid_lat, bbox.lon_min);
            0.8 * haversine(west, LatLon::new(mid_lat, bbox.lon_max))
        })
        .max(0.0);

    // Total tracked = number of (centre, bearing) candidates the search will
    // visit. The search bumps this counter once per candidate, accept or
    // reject — gives the FE a meaningful "evaluated N/M placements" readout.
    let progress_total = {
        // Mirror the candidate count expansion in search_text_placements:
        // 5 bearings per sampled position, ceil-divide to fit n_candidates.
        let bearings = 5usize;
        let n_pos = n_candidates.div_ceil(bearings).max(1);
        n_pos * bearings
    };
    let inner = state.0.clone();
    let search = Search::begin(&inner, "text", progress_total);
    let task = search.task();
    let text_owned = req.text.clone();
    let placements = tokio::task::spawn_blocking(move || -> Vec<text::MessagePlacement> {
        text::search_text_placements(
            &inner.graph,
            &text_owned,
            req.letter_height_m,
            wrap_m,
            font,
            &params,
            bbox,
            detour_cap,
            n_candidates,
            seed,
            Some(&task.done),
            Some(&task.cancel),
            &inner.no_go,
        )
    })
    .await;
    let placements = placements.map_err(AppError::internal)?;
    if search.cancelled() {
        return Err(AppError::cancelled());
    }

    // Probe layout to surface missing-glyph count.
    let probe = text::layout_text(
        &req.text,
        LatLon::new(
            (bbox.lat_min + bbox.lat_max) * 0.5,
            (bbox.lon_min + bbox.lon_max) * 0.5,
        ),
        std::f64::consts::FRAC_PI_2,
        req.letter_height_m,
        wrap_m,
        font,
    );

    let Some(best) = placements.into_iter().next() else {
        return Ok(Json(json!({
            "seed": seed,
            "results": Vec::<Value>::new(),
            "failures": json!({
                "snap_too_far": 0, "corridor_violated": 0, "no_route": 0,
                "no_go": 0, "too_short": 0, "too_long": 0, "other": 0,
                "total": progress_total,
            }),
            "text_render_meta": {
                "evaluated": progress_total,
                "missing_glyphs": probe.missing_glyphs,
                "hint": "no place found where every letter can be drawn closely enough — try bigger letters or a shorter message",
            },
        })));
    };

    let total_routed_m = best.total_routed_m;
    let total_intended_m = best.total_intended_m;
    let total_pen_up_m = best.total_pen_up_m;
    let detour = if total_intended_m > 0.0 {
        (total_routed_m / total_intended_m - 1.0).max(0.0)
    } else {
        0.0
    };
    let score = text::text_score(total_routed_m, total_intended_m, probe.missing_glyphs);

    // Surface breakdown spans both letter strokes and pen-up segments —
    // pen-ups are part of the rideable track, so treat them the same.
    let mut surface = [0.0_f64; 6];
    let mut ferry_km = 0.0;
    let mut accumulate_edge = |e: &crate::osm::EdgeData| {
        let s = (e.surface as usize).min(5);
        surface[s] += e.length_m / 1000.0;
        if (e.modes & crate::osm::MODE_FERRY) != 0 {
            ferry_km += e.length_m / 1000.0;
        }
    };
    for letter in &best.letters {
        for sr in &letter.stroke_routes {
            for e in &sr.edges {
                accumulate_edge(e);
            }
        }
    }

    // Build per-letter JSON (each letter contains its character + array of
    // routed strokes) and a parallel array of pen-up polylines. The FE
    // renders strokes thick, pen-ups thin/dashed.
    let letters_json: Vec<Value> = best
        .letters
        .iter()
        .map(|l| {
            let strokes: Vec<Vec<[f64; 2]>> = l
                .stroke_routes
                .iter()
                .map(|r| r.points.iter().map(|p| [p.lon, p.lat]).collect())
                .collect();
            json!({
                "char": l.ch.to_string(),
                "strokes": strokes,
                "center": [l.centre_chosen.lon, l.centre_chosen.lat],
                "bearing_deg": l.bearing_rad.to_degrees(),
                "max_stroke_detour_pct": ((l.max_stroke_detour - 1.0) * 100.0).max(0.0).round(),
            })
        })
        .collect();

    let pen_ups_json: Vec<Vec<[f64; 2]>> = best
        .pen_ups
        .iter()
        .map(|p| p.points.iter().map(|q| [q.lon, q.lat]).collect())
        .collect();

    // The whole rideable track in ride order: strokes interleaved with the
    // pen-ups between them (letter[0].stroke[0], pen_up[0], …). It is the
    // result's `coords` (map fit, GPX) and what the elevation profile samples.
    let mut ride: Vec<LatLon> = Vec::new();
    let mut pen_ups = best.pen_ups.iter();
    for (i, sr) in best
        .letters
        .iter()
        .flat_map(|l| &l.stroke_routes)
        .enumerate()
    {
        if i > 0 {
            if let Some(pu) = pen_ups.next() {
                ride.extend_from_slice(&pu.points);
            }
        }
        ride.extend_from_slice(&sr.points);
    }
    let all_coords: Vec<[f64; 2]> = ride.iter().map(|p| [p.lon, p.lat]).collect();

    let stroke_count: usize = best.letters.iter().map(|l| l.stroke_routes.len()).sum();
    let total_track_m = total_routed_m + total_pen_up_m;

    let elevation = state
        .0
        .dem
        .as_ref()
        .filter(|_| req.elevation_samples > 0)
        .and_then(|dem| profile_from_dem(dem, &ride, req.elevation_samples));

    let result = json!({
        "rank": 1,
        "score": score,
        "kind": "text",
        "text": req.text,
        "start": [best.anchor_centre.lon, best.anchor_centre.lat],
        "end":   [best.anchor_centre.lon, best.anchor_centre.lat],
        "bearing_deg": best.anchor_bearing_rad.to_degrees(),
        "ref_km": total_intended_m / 1000.0,
        "real_km": total_track_m / 1000.0,
        "max_dev_m": 0.0,
        "ferry_km": ferry_km,
        "surface": surface_json(&surface),
        "elevation": elevation,
        "coords": all_coords,
        "corridor": Vec::<[f64; 2]>::new(),
        "width_m": stroke_half_m * 2.0,
        "corridor_used_m": stroke_half_m * 2.0,
        "text_render": {
            "letters": letters_json,
            "pen_ups": pen_ups_json,
            "stroke_count": stroke_count,
            "pen_up_count": best.pen_ups.len(),
            "skipped_strokes": 0,
            "missing_glyphs": probe.missing_glyphs,
            "letter_height_m": req.letter_height_m,
            "wrap_m": wrap_m,
            "bbox_along_m": best.bbox_along_m,
            "bbox_across_m": best.bbox_across_m,
            "detour_pct": (detour * 100.0).round(),
            "max_stroke_detour_pct": ((best.max_stroke_detour - 1.0) * 100.0).max(0.0).round(),
            "pen_up_km": total_pen_up_m / 1000.0,
            "stroke_km": total_routed_m / 1000.0,
            "candidates_evaluated": progress_total,
            "font": req.font,
            "center": [best.anchor_centre.lon, best.anchor_centre.lat],
            "bearing_deg": best.anchor_bearing_rad.to_degrees(),
        },
    });

    Ok(Json(json!({
        "seed": seed,
        "results": [result],
        "failures": RejectionStats::default().to_json(),
    })))
}

/// Fill in elevation profiles from the resident DEM. Pure CPU (bilinear grid
/// lookups every 50 m), so callers run it inside their blocking task and
/// rayon spreads it across the pool. No-op without a DEM or when the request
/// asked for 0 samples.
fn enrich_scored(dem: Option<&Dem>, scored: &mut [Scored], n_samples: usize) {
    let Some(dem) = dem.filter(|_| n_samples > 0) else {
        return;
    };
    scored
        .par_iter_mut()
        .for_each(|s| s.elevation = profile_from_dem(dem, &s.route.points, n_samples));
}

/// Shape-search counterpart of [`enrich_scored`].
fn enrich_shapes(dem: Option<&Dem>, shapes: &mut [ScoredShape], n_samples: usize) {
    let Some(dem) = dem.filter(|_| n_samples > 0) else {
        return;
    };
    shapes
        .par_iter_mut()
        .for_each(|s| s.elevation = profile_from_dem(dem, &s.route.points, n_samples));
}

async fn handle_progress(State(state): State<AppState>) -> Json<Value> {
    let p = state.0.progress.lock().unwrap();
    match &*p {
        Some(pt) => Json(json!({
            "active": true,
            "kind": pt.kind,
            "done": pt.done.load(Ordering::Relaxed),
            "total": pt.total,
            "started_ms": pt.started_ms,
        })),
        None => Json(json!({ "active": false })),
    }
}

/// The request's seed, or a fresh random one so each click gives new
/// candidates. Responses echo it back for permalinks; drawing from u32 keeps
/// links short and the value exact as a JS number.
fn seed_or_random(seed: Option<u64>) -> u64 {
    seed.unwrap_or_else(|| u64::from(rand::random::<u32>()))
}

fn now_ms() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0)
}

#[derive(Deserialize)]
pub struct ElevationsReq {
    /// Route coordinates as `[lon, lat]`, as returned in a result's `coords`.
    pub coords: Vec<[f64; 2]>,
}

/// Per-point DEM elevations (0.1 m) for GPX `<ele>` tags. `ele` is null when
/// no DEM is loaded; single points outside DEM coverage are null.
async fn handle_elevations(
    State(state): State<AppState>,
    Json(req): Json<ElevationsReq>,
) -> Json<Value> {
    let Some(dem) = state.0.dem.as_ref() else {
        return Json(json!({ "ele": null }));
    };
    let ele: Vec<Option<f64>> = req
        .coords
        .iter()
        .map(|c| {
            dem.elevation_at(LatLon::new(c[1], c[0]))
                .map(|z| (z * 10.0).round() / 10.0)
        })
        .collect();
    Json(json!({ "ele": ele }))
}

/// Cancel the running search: the FE's Cancel button, or the tab closing.
/// Workers notice within one candidate (one shape leg); the search's own
/// request then answers 409.
async fn handle_cancel(State(state): State<AppState>) -> StatusCode {
    state
        .0
        .search_cancel
        .lock()
        .unwrap()
        .store(true, Ordering::Relaxed);
    StatusCode::NO_CONTENT
}

async fn handle_info(State(state): State<AppState>) -> Json<Value> {
    let b = state.0.bbox;
    let center = LatLon::new((b.lat_min + b.lat_max) * 0.5, (b.lon_min + b.lon_max) * 0.5);
    Json(json!({
        "bbox": [b.lon_min, b.lat_min, b.lon_max, b.lat_max],
        "center": [center.lon, center.lat],
        "graph_nodes": state.0.graph.graph.node_count(),
    }))
}

async fn handle_index() -> Response {
    (
        [(header::CONTENT_TYPE, "text/html; charset=utf-8")],
        INDEX_HTML,
    )
        .into_response()
}

async fn handle_style() -> Response {
    (
        [(header::CONTENT_TYPE, "text/css; charset=utf-8")],
        STYLE_CSS,
    )
        .into_response()
}

async fn handle_app_js() -> Response {
    javascript(APP_JS)
}

async fn handle_basemap_js() -> Response {
    javascript(BASEMAP_JS)
}

fn javascript(body: &'static str) -> Response {
    (
        [(
            header::CONTENT_TYPE,
            "application/javascript; charset=utf-8",
        )],
        body,
    )
        .into_response()
}

/// Build the axum router given a fully-prepared `AppState`.
pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(handle_index))
        .route("/style.css", get(handle_style))
        .route("/app.js", get(handle_app_js))
        .route("/basemap.js", get(handle_basemap_js))
        .route("/api/info", get(handle_info))
        .route("/api/progress", get(handle_progress))
        .route("/api/cancel", post(handle_cancel))
        .route("/api/between", post(handle_between))
        .route("/api/country", post(handle_country))
        .route("/api/shape-search", post(handle_shape_search))
        .route("/api/text", post(handle_text))
        // A long route's coords can exceed axum's 2 MB default body limit.
        .route(
            "/api/elevations",
            post(handle_elevations).layer(DefaultBodyLimit::max(32 * 1024 * 1024)),
        )
        .with_state(state)
}

/// Convenience: bind, log, serve.
pub async fn serve(state: AppState, bind: &str) -> Result<()> {
    let app = router(state);
    let listener = TcpListener::bind(bind).await?;
    let local = listener.local_addr()?;
    eprintln!(
        "\n  ⇨  Open  http://{}/  in your browser  (Ctrl-C to stop)\n",
        local
    );
    axum::serve(listener, app).await?;
    Ok(())
}

// ---- error handling ----

pub struct AppError {
    code: StatusCode,
    msg: String,
}

impl AppError {
    fn bad<E: std::fmt::Display>(e: E) -> Self {
        Self {
            code: StatusCode::BAD_REQUEST,
            msg: e.to_string(),
        }
    }
    fn internal<E: std::fmt::Display>(e: E) -> Self {
        Self {
            code: StatusCode::INTERNAL_SERVER_ERROR,
            msg: e.to_string(),
        }
    }
    /// The search was cancelled or superseded by a newer one.
    fn cancelled() -> Self {
        Self {
            code: StatusCode::CONFLICT,
            msg: "search cancelled".into(),
        }
    }
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        (self.code, Json(json!({ "error": self.msg }))).into_response()
    }
}

fn pick_alphas(n: usize) -> Vec<f64> {
    match n {
        1 => vec![4.0],
        2 => vec![1.0, 8.0],
        3 => vec![0.5, 4.0, 12.0],
        4 => vec![0.0, 2.0, 6.0, 14.0],
        _ => {
            // n alphas geometrically spaced from 0.25 to 16
            (0..n)
                .map(|i| {
                    let t = (i as f64) / ((n - 1) as f64);
                    0.25_f64 * (16.0_f64 / 0.25_f64).powf(t)
                })
                .collect()
        }
    }
}

const INDEX_HTML: &str = include_str!("../assets/index.html");
const STYLE_CSS: &str = include_str!("../assets/style.css");
const APP_JS: &str = include_str!("../assets/app.js");
const BASEMAP_JS: &str = include_str!("../assets/basemap.js");

#[cfg(test)]
mod tests {
    use super::*;

    /// Buckets match on message text, so pin them to the messages the
    /// routing code actually produces (see explore.rs / route.rs).
    #[test]
    fn rejections_land_in_the_right_bucket() {
        let mut s = RejectionStats::default();
        for msg in [
            "endpoint too far from graph: start 3100 m, end 20 m (tolerance 2000 m)",
            "leg 3: snap displacement too large (2500 m / 10 m)",
            "max deviation 4100 m exceeds corridor half-width 4000 m",
            "no-go: national_park",
            "routed distance 12.0 km below minimum 16.0 km",
            "routed distance 31.0 km above maximum 24.0 km",
            "no route found between selected nodes — try a wider corridor or different modes",
            "leg 2: detour 150% exceeds shape fidelity cap 120%",
            "cancelled",
        ] {
            s.classify(msg);
        }
        let j = s.to_json();
        for (bucket, n) in [
            ("snap_too_far", 2),
            ("corridor_violated", 1),
            ("no_go", 1),
            ("too_short", 1),
            ("too_long", 1),
            ("no_route", 1),
            ("too_winding", 1),
            ("other", 1),
            ("total", 9),
        ] {
            assert_eq!(j[bucket], n, "{bucket}");
        }
    }
}
