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

use crate::elevation::ElevationClient;
use crate::explore::{self, Candidate, RouteFilter, Scored, ScoredShape};
use crate::geodesy::{corridor_polygon, haversine, LatLon};
use crate::hershey_fonts::Font;
use crate::osm::{BBox, Graph, MODE_BIKE, MODE_FOOT, MODE_MTB, SURFACE_LABELS};
use crate::route::RouteParams;
use crate::text;
use crate::viability::NoGoZone;
use anyhow::{anyhow, Result};
use axum::{
    extract::{Path as AxumPath, State},
    http::{header, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use serde::Deserialize;
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;

pub struct ServerState {
    pub graph: Graph,
    pub bbox: BBox,
    pub no_go: Vec<NoGoZone>,
    pub long_edge_threshold_m: f64,
    pub elevation_cache: PathBuf,
    /// Mutex around the elevation client (it has on-disk cache and a "go offline
    /// after first failure" flag, so we serialize access).
    pub elev: Mutex<ElevationClient>,
    /// Latest in-flight search progress. Cleared when no work is running. The
    /// server is single-user (README), so a single slot is enough.
    pub progress: Mutex<Option<ProgressTracker>>,
    /// Slot for the most recent async job (background elevation + re-rank).
    /// Single-user assumption: only one outstanding job at a time. Cleared
    /// when a new search starts; the FE polls `/api/results/:id` to learn
    /// when re-ranking is finished and pick up the final order.
    pub async_job: Mutex<Option<AsyncJob>>,
}

/// Bookkeeping for one in-flight async re-rank. `kind` carries the result
/// pool typed for either country or shape responses; the BG task fills in
/// `final_*` once elevation enrichment + re-dedup completes.
pub struct AsyncJob {
    pub id: String,
    pub width_m: f64,
    pub completed: bool,
    pub failures: Value,
    pub kind: AsyncJobKind,
}

pub enum AsyncJobKind {
    Country {
        final_results: Option<Vec<Scored>>,
    },
    Shape {
        final_results: Option<Vec<ScoredShape>>,
    },
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
        self.other += other.other;
    }
    pub fn total(&self) -> usize {
        self.snap_too_far
            + self.corridor_violated
            + self.no_route
            + self.no_go
            + self.too_short
            + self.too_long
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
            "other":             self.other,
            "total":             self.total(),
        })
    }
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
    #[serde(default = "default_dist_min")]
    pub distance_min_km: f64,
    #[serde(default = "default_dist_max")]
    pub distance_max_km: f64,
    #[serde(default)]
    pub seed: Option<u64>,
    #[serde(default = "default_elev_samples")]
    pub elevation_samples: usize,
    /// Restrict to lines whose endpoints both fall inside the border strip,
    /// on different sides of the bbox.
    #[serde(default)]
    pub border_to_border: bool,
    #[serde(default = "default_border_strip")]
    pub border_strip_km: f64,
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
    /// many metres. 0 disables wrapping.
    #[serde(default = "default_text_wrap")]
    pub wrap_m: f64,
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
fn default_text_wrap() -> f64 {
    100_000.0
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

fn default_border_strip() -> f64 {
    25.0
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

fn parse_modes(s: &str) -> Result<u8> {
    let mut m = 0u8;
    for tok in s.split(',') {
        match tok.trim().to_ascii_lowercase().as_str() {
            "foot" | "walk" | "hike" => m |= MODE_FOOT,
            "bike" | "cycle" | "bicycle" => m |= MODE_BIKE,
            "mtb" | "gravel" => m |= MODE_MTB,
            "" => {}
            other => return Err(anyhow!("unknown mode: {other}")),
        }
    }
    if m == 0 {
        return Err(anyhow!("no modes"));
    }
    Ok(m)
}

fn scored_to_json(s: &Scored, rank: usize, width_m: f64) -> Value {
    let r = &s.report;
    let coords: Vec<[f64; 2]> = s.route.points.iter().map(|p| [p.lon, p.lat]).collect();
    let corridor: Vec<[f64; 2]> = corridor_polygon(s.candidate.start, s.candidate.end, width_m)
        .into_iter()
        .map(|p| [p.lon, p.lat])
        .collect();
    let surface_breakdown: Vec<Value> = SURFACE_LABELS
        .iter()
        .enumerate()
        .filter(|(i, _)| r.surface_km[*i] > 0.001)
        .map(|(i, label)| {
            json!({
                "label": label,
                "km": r.surface_km[i],
            })
        })
        .collect();
    let elev = s.elevation.as_ref().map(|p| {
        json!({
            "samples_m": p.samples_m,
            "ascent_m": p.ascent_m,
            "descent_m": p.descent_m,
            "max_m": p.max_m,
            "min_m": p.min_m,
        })
    });
    let ferry_km = r.ferry_km;
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
        "ferry_km": ferry_km,
        "surface": surface_breakdown,
        "elevation": elev,
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
    let modes = parse_modes(&req.modes).map_err(AppError::bad)?;
    let start = LatLon::new(req.start[0], req.start[1]);
    let end = LatLon::new(req.end[0], req.end[1]);
    let half = req.width_m / 2.0;

    // Vary alpha to get qualitatively different routes.
    let alphas = pick_alphas(req.alternatives.max(1));
    let inner = state.0.clone();

    let total = alphas.len();
    let counter = Arc::new(AtomicUsize::new(0));
    {
        let mut p = inner.progress.lock().unwrap();
        *p = Some(ProgressTracker {
            kind: "between",
            total,
            done: counter.clone(),
            started_ms: now_ms(),
        });
    }
    let counter_inner = counter.clone();
    let paved_only = req.paved_only;
    let scored_result = tokio::task::spawn_blocking(move || -> (Vec<Scored>, Vec<String>) {
        let mut out = Vec::with_capacity(alphas.len());
        let mut errors: Vec<String> = Vec::new();
        let filter = RouteFilter::lax(half.max(2_000.0));
        for a in alphas {
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
            counter_inner.fetch_add(1, Ordering::Relaxed);
        }
        (out, errors)
    })
    .await;
    let (scored, errors) = scored_result.map_err(AppError::internal)?;

    if scored.is_empty() {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
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

    let mut deduped = explore::rank_dedup_top(scored, req.alternatives, 0.5);
    enrich_elevation(&state, &mut deduped, req.elevation_samples).await;

    let width_m = req.width_m;
    let resp: Vec<Value> = deduped
        .iter()
        .enumerate()
        .map(|(i, s)| scored_to_json(s, i + 1, width_m))
        .collect();
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
    }
    Ok(Json(json!({ "results": resp })))
}

async fn handle_country(
    State(state): State<AppState>,
    Json(req): Json<CountryReq>,
) -> Result<Json<Value>, AppError> {
    let modes = parse_modes(&req.modes).map_err(AppError::bad)?;
    let base_half = req.width_m / 2.0;
    let count = req.count.clamp(1, 50);
    let base_sample = (count * 6).max(20); // oversample, then dedupe to top N
    let seed = req.seed.unwrap_or_else(|| {
        // Use system time so refresh gives new lines each click.
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0)
    });
    let inner = state.0.clone();
    let bbox = inner.bbox;
    let dmin = req.distance_min_km;
    let dmax = req.distance_max_km;
    let border_strip = req.border_strip_km;
    let border = req.border_to_border;
    let long_edge = inner.long_edge_threshold_m;
    let min_viable = req.min_viable.unwrap_or(count).max(1);
    // Persistence schedule: more batches than before, oversample grows late so
    // the first few attempts stay snappy, and the corridor widens by an
    // absolute delta (km of full corridor) so the worst case is request +2 km
    // — never a multiplicative blow-up. Strict mode zeroes out the deltas.
    const SAMPLE_FACTORS: [f64; 8] = [1.0, 1.0, 1.5, 1.5, 2.0, 2.0, 2.5, 2.5];
    const WIDEN_DELTA_KM: [f64; 8] = [0.0, 0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.0];
    const MAX_BATCHES: usize = SAMPLE_FACTORS.len();
    let strict = req.strict_corridor;

    let counter = Arc::new(AtomicUsize::new(0));
    {
        let mut p = inner.progress.lock().unwrap();
        *p = Some(ProgressTracker {
            kind: "country",
            total: 0,
            done: counter.clone(),
            started_ms: now_ms(),
        });
    }
    let counter_inner = counter.clone();
    let inner_for_task = inner.clone();
    let paved_only = req.paved_only;
    let scored_result = tokio::task::spawn_blocking(move || -> (Vec<Scored>, RejectionStats) {
        let mut all: Vec<Scored> = Vec::new();
        let mut stats = RejectionStats::default();
        for batch in 0..MAX_BATCHES {
            let delta_km = if strict { 0.0 } else { WIDEN_DELTA_KM[batch] };
            let sample_mul = SAMPLE_FACTORS[batch];
            // Half-width grows by half of the full-corridor delta.
            let half = base_half + delta_km * 500.0;
            let n_sample = ((base_sample as f64) * sample_mul).round() as usize;
            let params = RouteParams {
                modes,
                half_width_m: half,
                alpha: 4.0,
                corridor_max_m: half,
                paved_only,
            };
            let filter = RouteFilter {
                distance_min_km: Some(dmin),
                distance_max_km: Some(dmax),
                snap_tolerance_m: half.max(2_000.0),
                enforce_corridor: true,
            };
            // Each batch uses a different seed so we don't redraw the same
            // candidates we already proved infeasible.
            let batch_seed = seed.wrapping_add((batch as u64).wrapping_mul(0x9E3779B97F4A7C15));
            let candidates = if border {
                explore::generate_border_to_border_candidates(
                    bbox,
                    n_sample,
                    dmin,
                    dmax,
                    border_strip,
                    batch_seed,
                )
            } else {
                explore::generate_country_candidates_in_bbox(bbox, n_sample, dmin, dmax, batch_seed)
            };
            if candidates.is_empty() {
                continue;
            }
            // Publish the new batch's size before evaluation so the progress
            // bar's denominator grows visibly when we retry.
            {
                let mut pl = inner_for_task.progress.lock().unwrap();
                if let Some(pt) = pl.as_mut() {
                    pt.total += candidates.len();
                }
            }
            let results = explore::evaluate(
                &inner_for_task.graph,
                candidates,
                &inner_for_task.no_go,
                &params,
                &filter,
                long_edge,
                Some(&counter_inner),
            );
            for r in results {
                match r {
                    Ok(s) => all.push(s),
                    Err((_, msg)) => stats.classify(&msg),
                }
            }
            if all.len() >= min_viable {
                break;
            }
        }
        (all, stats)
    })
    .await;
    let (scored, stats) = scored_result.map_err(AppError::internal)?;

    if scored.is_empty() {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
        return Ok(Json(json!({
            "results": Vec::<Value>::new(),
            "failures": stats.to_json(),
        })));
    }

    // Per-section dedup with an over-keep pool so the BG elevation re-rank
    // has alternatives to promote. The synchronous response shows the top
    // `count` of each section by *pre-elevation* score.
    let pool_size = if req.elevation_samples > 0 {
        (count * 3).max(count + 4)
    } else {
        count
    };
    let width_m = req.width_m;
    let (tight, loose): (Vec<Scored>, Vec<Scored>) = scored
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let tight_pool = explore::rank_dedup_top(tight, pool_size, 8.0);
    let loose_pool = explore::rank_dedup_top(loose, pool_size, 8.0);

    // Initial display set (no elevation): top `count` of each section.
    let mut initial: Vec<Scored> = tight_pool.iter().take(count).cloned().collect();
    initial.extend(loose_pool.iter().take(count).cloned());
    let resp_initial: Vec<Value> = initial
        .iter()
        .enumerate()
        .map(|(i, s)| scored_to_json(s, i + 1, width_m))
        .collect();

    // Set up an async job for elevation enrichment + re-rank. The FE polls
    // /api/results/:id and animates cards into the new order on completion.
    let elevation_pending = req.elevation_samples > 0;
    let request_id = next_async_id();
    let failures_json = stats.to_json();
    if elevation_pending {
        {
            let mut slot = state.0.async_job.lock().unwrap();
            *slot = Some(AsyncJob {
                id: request_id.clone(),
                width_m,
                completed: false,
                failures: failures_json.clone(),
                kind: AsyncJobKind::Country {
                    final_results: None,
                },
            });
        }
        let state_for_bg = state.clone();
        let request_id_bg = request_id.clone();
        let elevation_samples = req.elevation_samples;
        tokio::spawn(async move {
            run_country_rerank(
                state_for_bg,
                request_id_bg,
                tight_pool,
                loose_pool,
                count,
                width_m,
                elevation_samples,
            )
            .await;
        });
    } else {
        // No elevation requested → clear progress, no async job.
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
    }

    Ok(Json(json!({
        "request_id": request_id,
        "elevation_pending": elevation_pending,
        "results": resp_initial,
        "failures": failures_json,
    })))
}

/// Background re-rank for country mode: enrich the entire over-keep pool with
/// elevation, recompute scores, dedup again per-section, and store the final
/// order in the async-job slot for the FE to poll.
async fn run_country_rerank(
    state: AppState,
    request_id: String,
    tight_pool: Vec<Scored>,
    loose_pool: Vec<Scored>,
    count: usize,
    width_m: f64,
    elevation_samples: usize,
) {
    let mut combined: Vec<Scored> = Vec::with_capacity(tight_pool.len() + loose_pool.len());
    combined.extend(tight_pool);
    combined.extend(loose_pool);
    enrich_elevation(&state, &mut combined, elevation_samples).await;
    for s in combined.iter_mut() {
        explore::rescore_with_elevation(s);
    }
    let (tight2, loose2): (Vec<Scored>, Vec<Scored>) = combined
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let tight_final = explore::rank_dedup_top(tight2, count, 8.0);
    let loose_final = explore::rank_dedup_top(loose2, count, 8.0);
    let mut all_final: Vec<Scored> = tight_final;
    all_final.extend(loose_final);
    {
        let mut slot = state.0.async_job.lock().unwrap();
        if let Some(job) = slot.as_mut() {
            if job.id == request_id {
                if let AsyncJobKind::Country { final_results } = &mut job.kind {
                    *final_results = Some(all_final);
                }
                job.completed = true;
            }
        }
    }
    let mut p = state.0.progress.lock().unwrap();
    *p = None;
}

fn next_async_id() -> String {
    use std::sync::atomic::AtomicU64;
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{}-{}", now_ms(), n)
}

fn shape_to_json(s: &ScoredShape, rank: usize, width_m: f64) -> Value {
    let coords: Vec<[f64; 2]> = s.route.points.iter().map(|p| [p.lon, p.lat]).collect();
    let surface_breakdown: Vec<Value> = SURFACE_LABELS
        .iter()
        .enumerate()
        .filter(|(i, _)| s.surface_km[*i] > 0.001)
        .map(|(i, label)| json!({ "label": label, "km": s.surface_km[i] }))
        .collect();
    let elev = s.elevation.as_ref().map(|p| {
        json!({
            "samples_m": p.samples_m,
            "ascent_m": p.ascent_m,
            "descent_m": p.descent_m,
            "max_m": p.max_m,
            "min_m": p.min_m,
        })
    });
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
        "surface": surface_breakdown,
        "elevation": elev,
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
    let modes = parse_modes(&req.modes).map_err(AppError::bad)?;
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
    let seed = req.seed.unwrap_or_else(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0)
    });
    let min_viable = req.min_viable.unwrap_or(count).max(1);
    const SAMPLE_FACTORS: [f64; 5] = [1.0, 1.5, 2.0, 2.5, 3.0];
    const WIDEN_DELTA_KM: [f64; 5] = [0.0, 0.5, 1.0, 1.5, 2.0];
    const MAX_BATCHES: usize = SAMPLE_FACTORS.len();
    let strict = req.strict_corridor;

    let counter = Arc::new(AtomicUsize::new(0));
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = Some(ProgressTracker {
            kind: "shape-search",
            total: 0,
            done: counter.clone(),
            started_ms: now_ms(),
        });
    }

    let inner = state.0.clone();
    let inner_for_task = inner.clone();
    let counter_inner = counter.clone();
    let bbox = inner.bbox;
    let perimeter_km = req.perimeter_km;
    let paved_only = req.paved_only;
    let routed = tokio::task::spawn_blocking(move || -> (Vec<ScoredShape>, RejectionStats) {
        let mut all: Vec<ScoredShape> = Vec::new();
        let mut stats = RejectionStats::default();
        for batch in 0..MAX_BATCHES {
            let batch_started = std::time::Instant::now();
            let delta_km = if strict { 0.0 } else { WIDEN_DELTA_KM[batch] };
            let sample_mul = SAMPLE_FACTORS[batch];
            let half = base_half + delta_km * 500.0;
            let n_sample = ((base_sample as f64) * sample_mul).round() as usize;
            let params = RouteParams {
                modes,
                half_width_m: half,
                alpha: 4.0,
                corridor_max_m: half,
                paved_only,
            };
            let filter = RouteFilter {
                distance_min_km: None,
                distance_max_km: None,
                snap_tolerance_m: half.max(2_000.0),
                enforce_corridor: true,
            };
            // Abort bad shape candidates early. A single leg >120% longer
            // than intended creates the visible tails/kinks we don't want, and
            // continuing to route the remaining 20-30 legs is wasted work.
            let max_leg_detour_cap = Some(1.2 + delta_km * 0.15);
            let batch_seed = seed.wrapping_add((batch as u64).wrapping_mul(0x9E3779B97F4A7C15));
            let candidates = explore::generate_shape_candidates_in_bbox(
                bbox,
                kind,
                perimeter_km,
                n_sample,
                batch_seed,
            );
            if candidates.is_empty() {
                break; // shape doesn't fit in bbox
            }
            {
                let mut pl = inner_for_task.progress.lock().unwrap();
                if let Some(pt) = pl.as_mut() {
                    pt.total += candidates.len();
                }
            }
            let results = explore::evaluate_shapes_with_limits(
                &inner_for_task.graph,
                candidates,
                &inner_for_task.no_go,
                &params,
                &filter,
                Some(&counter_inner),
                max_leg_detour_cap,
            );
            for r in results {
                match r {
                    Ok(s) => all.push(s),
                    Err((_, msg)) => stats.classify(&msg),
                }
            }
            eprintln!(
                "shape batch {}: {} candidates, {} viable total, {:.1}s",
                batch + 1,
                n_sample,
                all.len(),
                batch_started.elapsed().as_secs_f64()
            );
            if all.len() >= min_viable {
                break;
            }
        }
        (all, stats)
    })
    .await;
    let (scored, stats) = routed.map_err(AppError::internal)?;

    if scored.is_empty() {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
        return Ok(Json(json!({
            "results": Vec::<Value>::new(),
            "failures": stats.to_json(),
        })));
    }
    // Dedup by center proximity, scaled to the shape's diameter so two big
    // shapes don't merge unless they really overlap.
    let dedup_sep_km = (req.perimeter_km / std::f64::consts::TAU).max(8.0);
    // Over-keep ahead of elevation enrichment so the ascent re-score can
    // promote a flatter loop over a marginally tighter but climbier one.
    let pool_size = if req.elevation_samples > 0 {
        (count * 3).max(count + 4)
    } else {
        count
    };
    let width_m = req.width_m;
    let (tight, loose): (Vec<ScoredShape>, Vec<ScoredShape>) = scored
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let tight_pool = explore::rank_dedup_top_shapes(tight, pool_size, dedup_sep_km);
    let loose_pool = explore::rank_dedup_top_shapes(loose, pool_size, dedup_sep_km);

    // Initial display set (no elevation): top `count` of each section.
    let mut initial: Vec<ScoredShape> = tight_pool.iter().take(count).cloned().collect();
    initial.extend(loose_pool.iter().take(count).cloned());
    let resp_initial: Vec<Value> = initial
        .iter()
        .enumerate()
        .map(|(i, s)| shape_to_json(s, i + 1, width_m))
        .collect();

    // Spawn background elevation + re-rank (same async-job pattern as country).
    let elevation_pending = req.elevation_samples > 0;
    let request_id = next_async_id();
    let failures_json = stats.to_json();
    if elevation_pending {
        {
            let mut slot = state.0.async_job.lock().unwrap();
            *slot = Some(AsyncJob {
                id: request_id.clone(),
                width_m,
                completed: false,
                failures: failures_json.clone(),
                kind: AsyncJobKind::Shape {
                    final_results: None,
                },
            });
        }
        let state_for_bg = state.clone();
        let request_id_bg = request_id.clone();
        let elevation_samples = req.elevation_samples;
        tokio::spawn(async move {
            run_shape_rerank(
                state_for_bg,
                request_id_bg,
                tight_pool,
                loose_pool,
                count,
                width_m,
                dedup_sep_km,
                elevation_samples,
            )
            .await;
        });
    } else {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
    }

    Ok(Json(json!({
        "request_id": request_id,
        "elevation_pending": elevation_pending,
        "results": resp_initial,
        "failures": failures_json,
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
    let modes = parse_modes(&req.modes).map_err(AppError::bad)?;
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
    let seed = req.seed.unwrap_or_else(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0)
    });
    let bbox = state.0.bbox;
    let wrap_m = if req.wrap_m > 0.0 { req.wrap_m } else { 0.0 };

    let counter = Arc::new(AtomicUsize::new(0));
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
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = Some(ProgressTracker {
            kind: "text",
            total: progress_total,
            done: counter.clone(),
            started_ms: now_ms(),
        });
    }

    let inner = state.0.clone();
    let counter_inner = counter.clone();
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
            Some(&counter_inner),
            &inner.no_go,
        )
    })
    .await
    .map_err(AppError::internal)?;

    {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
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
            "results": Vec::<Value>::new(),
            "failures": json!({
                "snap_too_far": 0, "corridor_violated": 0, "no_route": 0,
                "no_go": 0, "too_short": 0, "too_long": 0, "other": 0,
                "total": progress_total,
            }),
            "text_render_meta": {
                "evaluated": progress_total,
                "missing_glyphs": probe.missing_glyphs,
                "hint": "no placement found where every letter routes within the detour cap — try larger letter_km, shorter text, or higher detour_cap",
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
    let surface_breakdown: Vec<Value> = SURFACE_LABELS
        .iter()
        .enumerate()
        .filter(|(i, _)| surface[*i] > 0.001)
        .map(|(i, label)| json!({ "label": label, "km": surface[i] }))
        .collect();

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

    // For map fitBounds + GPX flattening, concatenate letters + pen-ups in
    // ride order (letter[0].strokes ++ pen_up[0..k0] ++ letter[1].strokes …).
    let mut all_coords: Vec<[f64; 2]> = Vec::new();
    let mut pu_idx = 0usize;
    let mut first_stroke_overall = true;
    for letter in &best.letters {
        for sr in &letter.stroke_routes {
            if !first_stroke_overall {
                if let Some(pu) = best.pen_ups.get(pu_idx) {
                    for p in &pu.points {
                        all_coords.push([p.lon, p.lat]);
                    }
                }
                pu_idx += 1;
            }
            for p in &sr.points {
                all_coords.push([p.lon, p.lat]);
            }
            first_stroke_overall = false;
        }
    }

    let stroke_count: usize = best.letters.iter().map(|l| l.stroke_routes.len()).sum();
    let total_track_m = total_routed_m + total_pen_up_m;

    let elevation_json = if req.elevation_samples > 0 {
        let inner = state.0.clone();
        // Sample over the full rideable track (strokes + pen-ups in order)
        // so the elevation profile matches what someone actually rides.
        let pts: Vec<LatLon> = {
            let mut out: Vec<LatLon> = Vec::new();
            let mut pu_idx = 0usize;
            let mut first = true;
            for letter in &best.letters {
                for sr in &letter.stroke_routes {
                    if !first {
                        if let Some(pu) = best.pen_ups.get(pu_idx) {
                            out.extend_from_slice(&pu.points);
                        }
                        pu_idx += 1;
                    }
                    out.extend_from_slice(&sr.points);
                    first = false;
                }
            }
            out
        };
        let n = req.elevation_samples;
        let prof = tokio::task::spawn_blocking(move || -> Option<crate::elevation::Profile> {
            let mut client = inner.elev.lock().unwrap();
            let p = client.profile(&pts, n);
            let _ = client.save();
            p
        })
        .await
        .ok()
        .flatten();
        prof.map(|p| {
            json!({
                "samples_m": p.samples_m,
                "ascent_m": p.ascent_m,
                "descent_m": p.descent_m,
                "max_m": p.max_m,
                "min_m": p.min_m,
            })
        })
    } else {
        None
    };

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
        "surface": surface_breakdown,
        "elevation": elevation_json,
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
            "wrap_m": req.wrap_m,
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
        "results": [result],
        "failures": json!({
            "snap_too_far": 0, "corridor_violated": 0, "no_route": 0,
            "no_go": 0, "too_short": 0, "too_long": 0, "other": 0,
            "total": 0,
        }),
    })))
}

async fn run_shape_rerank(
    state: AppState,
    request_id: String,
    tight_pool: Vec<ScoredShape>,
    loose_pool: Vec<ScoredShape>,
    count: usize,
    width_m: f64,
    dedup_sep_km: f64,
    elevation_samples: usize,
) {
    let mut combined: Vec<ScoredShape> = Vec::with_capacity(tight_pool.len() + loose_pool.len());
    combined.extend(tight_pool);
    combined.extend(loose_pool);
    if !combined.is_empty() {
        let total = combined.len();
        let counter = Arc::new(AtomicUsize::new(0));
        {
            let mut p = state.0.progress.lock().unwrap();
            *p = Some(ProgressTracker {
                kind: "elevation",
                total,
                done: counter.clone(),
                started_ms: now_ms(),
            });
        }
        let inner = state.0.clone();
        let owned = std::mem::take(&mut combined);
        let counter_inner = counter.clone();
        let updated = tokio::task::spawn_blocking(move || -> Vec<ScoredShape> {
            let mut owned = owned;
            let mut client = inner.elev.lock().unwrap();
            for s in owned.iter_mut() {
                s.elevation = client.profile(&s.route.points, elevation_samples);
                counter_inner.fetch_add(1, Ordering::Relaxed);
            }
            let _ = client.save();
            owned
        })
        .await
        .unwrap_or_default();
        combined = updated;
        for s in combined.iter_mut() {
            explore::rescore_shape_with_elevation(s);
        }
    }
    let (tight2, loose2): (Vec<ScoredShape>, Vec<ScoredShape>) = combined
        .into_iter()
        .partition(|s| s.corridor_used_m <= width_m + 1.0);
    let tight_final = explore::rank_dedup_top_shapes(tight2, count, dedup_sep_km);
    let loose_final = explore::rank_dedup_top_shapes(loose2, count, dedup_sep_km);
    let mut all_final: Vec<ScoredShape> = tight_final;
    all_final.extend(loose_final);
    {
        let mut slot = state.0.async_job.lock().unwrap();
        if let Some(job) = slot.as_mut() {
            if job.id == request_id {
                if let AsyncJobKind::Shape { final_results } = &mut job.kind {
                    *final_results = Some(all_final);
                }
                job.completed = true;
            }
        }
    }
    let mut p = state.0.progress.lock().unwrap();
    *p = None;
}

async fn enrich_elevation(state: &AppState, scored: &mut Vec<Scored>, n_samples: usize) {
    if n_samples == 0 || scored.is_empty() {
        return;
    }
    let total = scored.len();
    let counter = Arc::new(AtomicUsize::new(0));
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = Some(ProgressTracker {
            kind: "elevation",
            total,
            done: counter.clone(),
            started_ms: now_ms(),
        });
    }
    let inner = state.0.clone();
    let owned = std::mem::take(scored);
    let counter_inner = counter.clone();
    let updated = tokio::task::spawn_blocking(move || -> Vec<Scored> {
        let mut owned = owned;
        let mut client = inner.elev.lock().unwrap();
        for s in owned.iter_mut() {
            s.elevation = client.profile(&s.route.points, n_samples);
            counter_inner.fetch_add(1, Ordering::Relaxed);
        }
        let _ = client.save();
        owned
    })
    .await
    .unwrap_or_default();
    *scored = updated;
}

async fn handle_results(
    State(state): State<AppState>,
    AxumPath(id): AxumPath<String>,
) -> Json<Value> {
    let slot = state.0.async_job.lock().unwrap();
    let Some(job) = slot.as_ref() else {
        return Json(json!({ "error": "no active async job" }));
    };
    if job.id != id {
        return Json(json!({ "error": "unknown request_id" }));
    }
    match &job.kind {
        AsyncJobKind::Country {
            final_results: Some(rs),
        } => {
            let resp: Vec<Value> = rs
                .iter()
                .enumerate()
                .map(|(i, s)| scored_to_json(s, i + 1, job.width_m))
                .collect();
            Json(json!({
                "request_id": job.id,
                "completed": true,
                "results": resp,
                "failures": job.failures,
            }))
        }
        AsyncJobKind::Shape {
            final_results: Some(rs),
        } => {
            let resp: Vec<Value> = rs
                .iter()
                .enumerate()
                .map(|(i, s)| shape_to_json(s, i + 1, job.width_m))
                .collect();
            Json(json!({
                "request_id": job.id,
                "completed": true,
                "results": resp,
                "failures": job.failures,
            }))
        }
        _ => Json(json!({
            "request_id": job.id,
            "completed": false,
        })),
    }
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

fn now_ms() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0)
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
    (
        [(
            header::CONTENT_TYPE,
            "application/javascript; charset=utf-8",
        )],
        APP_JS,
    )
        .into_response()
}

/// Build the axum router given a fully-prepared `AppState`.
pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(handle_index))
        .route("/style.css", get(handle_style))
        .route("/app.js", get(handle_app_js))
        .route("/api/info", get(handle_info))
        .route("/api/progress", get(handle_progress))
        .route("/api/results/:id", get(handle_results))
        .route("/api/between", post(handle_between))
        .route("/api/country", post(handle_country))
        .route("/api/shape-search", post(handle_shape_search))
        .route("/api/text", post(handle_text))
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
