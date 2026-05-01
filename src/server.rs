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
use crate::explore::{self, Candidate, RouteFilter, Scored};
use crate::geodesy::{corridor_polygon, haversine, LatLon};
use crate::osm::{BBox, Graph, MODE_BIKE, MODE_FOOT, MODE_MTB, SURFACE_LABELS};
use crate::route::RouteParams;
use crate::viability::NoGoZone;
use anyhow::{anyhow, Result};
use axum::{
    extract::State,
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
    let coords: Vec<[f64; 2]> = s
        .route
        .points
        .iter()
        .map(|p| [p.lon, p.lat])
        .collect();
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
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
    }
    let (scored, errors) = scored_result.map_err(AppError::internal)?;

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

    let mut deduped = explore::rank_dedup_top(scored, req.alternatives, 0.5);
    enrich_elevation(&state, &mut deduped, req.elevation_samples).await;

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
    let modes = parse_modes(&req.modes).map_err(AppError::bad)?;
    let half = req.width_m / 2.0;
    let count = req.count.clamp(1, 50);
    let n_sample = (count * 6).max(20); // oversample, then dedupe to top N
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
    // Initial batch + up to 3 retries when the viable count falls short.
    const MAX_BATCHES: u64 = 4;

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
    let scored_result = tokio::task::spawn_blocking(move || -> Vec<Scored> {
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
        let mut all: Vec<Scored> = Vec::new();
        for batch in 0..MAX_BATCHES {
            // Each batch uses a different seed so we don't redraw the same
            // candidates we already proved infeasible.
            let batch_seed = seed.wrapping_add(batch.wrapping_mul(0x9E3779B97F4A7C15));
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
                explore::generate_country_candidates_in_bbox(
                    bbox,
                    n_sample,
                    dmin,
                    dmax,
                    batch_seed,
                )
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
            all.extend(results.into_iter().filter_map(|r| r.ok()));
            if all.len() >= min_viable {
                break;
            }
        }
        all
    })
    .await;
    {
        let mut p = state.0.progress.lock().unwrap();
        *p = None;
    }
    let scored = scored_result.map_err(AppError::internal)?;

    if scored.is_empty() {
        return Err(AppError::bad(anyhow!(
            "no candidate produced a viable route — relax the corridor or distance range"
        )));
    }

    // Country lines are diverse by construction; dedup with a generous
    // separation to avoid serving near-duplicates.
    let mut deduped = explore::rank_dedup_top(scored, count, 8.0);
    enrich_elevation(&state, &mut deduped, req.elevation_samples).await;

    let width_m = req.width_m;
    let resp: Vec<Value> = deduped
        .iter()
        .enumerate()
        .map(|(i, s)| scored_to_json(s, i + 1, width_m))
        .collect();
    Ok(Json(json!({ "results": resp })))
}

async fn enrich_elevation(state: &AppState, scored: &mut Vec<Scored>, n_samples: usize) {
    if n_samples == 0 || scored.is_empty() {
        return;
    }
    let inner = state.0.clone();
    let owned = std::mem::take(scored);
    let updated = tokio::task::spawn_blocking(move || -> Vec<Scored> {
        let mut owned = owned;
        let mut client = inner.elev.lock().unwrap();
        for s in owned.iter_mut() {
            s.elevation = client.profile(&s.route.points, n_samples);
        }
        let _ = client.save();
        owned
    })
    .await
    .unwrap_or_default();
    *scored = updated;
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
    let center = LatLon::new(
        (b.lat_min + b.lat_max) * 0.5,
        (b.lon_min + b.lon_max) * 0.5,
    );
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

/// Build the axum router given a fully-prepared `AppState`.
pub fn router(state: AppState) -> Router {
    Router::new()
        .route("/", get(handle_index))
        .route("/api/info", get(handle_info))
        .route("/api/progress", get(handle_progress))
        .route("/api/between", post(handle_between))
        .route("/api/country", post(handle_country))
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
