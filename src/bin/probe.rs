//! Probe driver for crowfly's bike routing.
//!
//! Fires a small fixed panel of A→B requests at a running `crowfly serve`
//! and computes geometric quality metrics (turn density, sharp turns,
//! hairpins, self-proximity loops, surface fragmentation, ascent rate) on
//! each returned route. Save runs as JSON for head-to-head comparison.
//!
//! Usage:
//!   cargo run --release --bin probe -- --label baseline
//!   cargo run --release --bin probe -- --label compressed --compare baseline

use anyhow::{Context, Result};
use crowfly::geodesy::{bearing_rad, haversine, LatLon};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::fs;
use std::path::PathBuf;
use std::time::Instant;

#[derive(Clone)]
struct PanelEntry {
    name: &'static str,
    start: [f64; 2], // [lat, lon]
    end: [f64; 2],
    width_m: f64,
}

const PANEL: &[PanelEntry] = &[
    PanelEntry { name: "zh_winti", start: [47.3769, 8.5417], end: [47.4995, 8.7240], width_m: 6_000.0 },
    PanelEntry { name: "bn_fr",    start: [46.9481, 7.4474], end: [46.8060, 7.1614], width_m: 8_000.0 },
    PanelEntry { name: "ls_vv",    start: [46.5197, 6.6323], end: [46.4628, 6.8419], width_m: 5_000.0 },
    PanelEntry { name: "zh_lu",    start: [47.3769, 8.5417], end: [47.0502, 8.3093], width_m: 8_000.0 },
    PanelEntry { name: "bn_thn",   start: [46.9481, 7.4474], end: [46.7494, 7.6280], width_m: 6_000.0 },
];

const MODES: &str = "foot,bike";
const ELEV_SAMPLES: usize = 80;

#[derive(Serialize, Deserialize, Clone)]
struct CaseMetrics {
    name: String,
    ref_km: f64,
    real_km: f64,
    detour_pct: f64,
    max_dev_m: f64,
    ascent_m: Option<f64>,
    ascent_per_km: Option<f64>,
    turn_density_dps_per_km: f64,
    sharp_turns_per_km: f64,
    hairpins_per_km: f64,
    detour_loops: usize,
    /// Tight self-proximity events that match the "weird tails" the user
    /// flagged: the polyline returns within 30 m of an earlier point while
    /// having walked at least 300 m in between. Graph-level loop pruning
    /// should drive this to zero on between-A→B; shape mode may still hit a
    /// few from terrain-forced switchbacks.
    #[serde(default)]
    tails: usize,
    surface_transitions_per_km: f64,
    elapsed_s: f64,
}

#[derive(Serialize, Deserialize, Clone)]
struct Run {
    label: String,
    host: String,
    results: Vec<CaseMetrics>,
}

fn coalesce(coords: &[[f64; 2]], min_step_m: f64) -> Vec<[f64; 2]> {
    if coords.is_empty() {
        return Vec::new();
    }
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(coords.len());
    out.push(coords[0]);
    for &p in &coords[1..] {
        let last = *out.last().unwrap();
        let d = haversine(LatLon::new(last[1], last[0]), LatLon::new(p[1], p[0]));
        if d >= min_step_m {
            out.push(p);
        }
    }
    if out.last() != coords.last() {
        out.push(*coords.last().unwrap());
    }
    out
}

fn bearing_deg_xy(a: [f64; 2], b: [f64; 2]) -> f64 {
    let r = bearing_rad(LatLon::new(a[1], a[0]), LatLon::new(b[1], b[0]));
    let mut d = r.to_degrees();
    if d < 0.0 {
        d += 360.0;
    }
    d
}

fn ang_diff(a: f64, b: f64) -> f64 {
    let mut d = (a - b).abs() % 360.0;
    if d > 180.0 {
        d = 360.0 - d;
    }
    d
}

/// (mean deg/km, sharp/km ≥60°, hairpins/km ≥110°)
fn turn_metrics(coords: &[[f64; 2]]) -> (f64, f64, f64) {
    if coords.len() < 3 {
        return (0.0, 0.0, 0.0);
    }
    let c = coalesce(coords, 15.0);
    if c.len() < 3 {
        return (0.0, 0.0, 0.0);
    }
    let mut total_deg = 0.0_f64;
    let mut total_m = 0.0_f64;
    let mut sharp = 0_usize;
    let mut hairpin = 0_usize;
    for i in 1..(c.len() - 1) {
        let a = c[i - 1];
        let b = c[i];
        let d = c[i + 1];
        let seg = haversine(LatLon::new(a[1], a[0]), LatLon::new(b[1], b[0]));
        total_m += seg;
        if seg < 5.0 {
            continue;
        }
        let theta = ang_diff(bearing_deg_xy(a, b), bearing_deg_xy(b, d));
        total_deg += theta;
        if theta >= 60.0 {
            sharp += 1;
        }
        if theta >= 110.0 {
            hairpin += 1;
        }
    }
    let km = (total_m / 1000.0).max(1e-6);
    (total_deg / km, sharp as f64 / km, hairpin as f64 / km)
}

/// Counts self-proximity events: pairs (i,j) where the polyline returns
/// within ~60 m of itself despite >1.5 km of along-line distance between
/// the two visits. Subsamples to keep the comparison ~O(n²/k²).
fn detour_loops(coords: &[[f64; 2]]) -> usize {
    if coords.len() < 200 {
        return 0;
    }
    let stride = (coords.len() / 600).max(1);
    let sampled: Vec<[f64; 2]> = coords.iter().step_by(stride).copied().collect();
    let mut cum = vec![0.0_f64; sampled.len()];
    for i in 1..sampled.len() {
        cum[i] = cum[i - 1]
            + haversine(
                LatLon::new(sampled[i - 1][1], sampled[i - 1][0]),
                LatLon::new(sampled[i][1], sampled[i][0]),
            );
    }
    let mut used = vec![false; sampled.len()];
    let mut loops = 0_usize;
    for i in 0..sampled.len() {
        if used[i] {
            continue;
        }
        let mut j = i + 30;
        while j < sampled.len() {
            if !used[j] {
                let d = haversine(
                    LatLon::new(sampled[i][1], sampled[i][0]),
                    LatLon::new(sampled[j][1], sampled[j][0]),
                );
                if d <= 60.0 {
                    let along = cum[j] - cum[i];
                    if along > 1500.0 {
                        loops += 1;
                        let lo = i.saturating_sub(2);
                        let hi = (j + 3).min(sampled.len());
                        for k in lo..hi {
                            used[k] = true;
                        }
                        break;
                    }
                }
            }
            j += 1;
        }
    }
    loops
}

/// Count "tails": points (i, j) with j > i+10 where d(p_i, p_j) < 30 m and
/// the walked distance between them is > 300 m. A clean polyline has none.
/// Mirrors `detour_loops` but with much tighter spatial / along-line
/// thresholds so it catches the narrow lobes that the user flagged.
fn tail_count(coords: &[[f64; 2]]) -> usize {
    if coords.len() < 20 {
        return 0;
    }
    let stride = (coords.len() / 800).max(1);
    let sampled: Vec<[f64; 2]> = coords.iter().step_by(stride).copied().collect();
    let mut cum = vec![0.0_f64; sampled.len()];
    for i in 1..sampled.len() {
        cum[i] = cum[i - 1]
            + haversine(
                LatLon::new(sampled[i - 1][1], sampled[i - 1][0]),
                LatLon::new(sampled[i][1], sampled[i][0]),
            );
    }
    let mut used = vec![false; sampled.len()];
    let mut tails = 0;
    for i in 0..sampled.len() {
        if used[i] {
            continue;
        }
        let mut j = i + 10;
        while j < sampled.len() {
            if !used[j] {
                let d = haversine(
                    LatLon::new(sampled[i][1], sampled[i][0]),
                    LatLon::new(sampled[j][1], sampled[j][0]),
                );
                if d <= 30.0 {
                    let along = cum[j] - cum[i];
                    if along > 300.0 {
                        tails += 1;
                        let lo = i.saturating_sub(1);
                        let hi = (j + 2).min(sampled.len());
                        for k in lo..hi {
                            used[k] = true;
                        }
                        break;
                    }
                }
            }
            j += 1;
        }
    }
    tails
}

fn surface_fragmentation(surface: &Value, real_km: f64) -> f64 {
    let arr = match surface.as_array() {
        Some(a) => a,
        None => return 0.0,
    };
    let significant = arr
        .iter()
        .filter(|s| s.get("km").and_then(|v| v.as_f64()).unwrap_or(0.0) >= 0.3)
        .count();
    let extras = significant.saturating_sub(1) as f64;
    extras / real_km.max(1.0)
}

fn metrics_from_result(name: &str, r: &Value, elapsed_s: f64) -> CaseMetrics {
    let coords: Vec<[f64; 2]> = r
        .get("coords")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|p| {
                    let arr = p.as_array()?;
                    Some([arr.first()?.as_f64()?, arr.get(1)?.as_f64()?])
                })
                .collect()
        })
        .unwrap_or_default();
    let ref_km = r.get("ref_km").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let real_km = r.get("real_km").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let max_dev_m = r.get("max_dev_m").and_then(|v| v.as_f64()).unwrap_or(0.0);
    let detour_pct = if ref_km > 0.0 {
        100.0 * (real_km / ref_km - 1.0)
    } else {
        0.0
    };
    let elev = r.get("elevation").cloned().unwrap_or(Value::Null);
    let ascent_m = elev.get("ascent_m").and_then(|v| v.as_f64());
    let ascent_per_km = ascent_m.and_then(|a| if real_km > 0.0 { Some(a / real_km) } else { None });
    let (td, sh, hp) = turn_metrics(&coords);
    let loops = detour_loops(&coords);
    let tails = tail_count(&coords);
    let surf = surface_fragmentation(r.get("surface").unwrap_or(&Value::Null), real_km);
    CaseMetrics {
        name: name.to_string(),
        ref_km,
        real_km,
        detour_pct,
        max_dev_m,
        ascent_m,
        ascent_per_km,
        turn_density_dps_per_km: td,
        sharp_turns_per_km: sh,
        hairpins_per_km: hp,
        detour_loops: loops,
        tails,
        surface_transitions_per_km: surf,
        elapsed_s,
    }
}

fn post(host: &str, path: &str, body: Value) -> Result<Value> {
    let url = format!("{}{}", host.trim_end_matches('/'), path);
    let resp = ureq::post(&url)
        .set("Content-Type", "application/json")
        .timeout(std::time::Duration::from_secs(180))
        .send_string(&body.to_string())
        .with_context(|| format!("POST {}", url))?;
    let s = resp.into_string()?;
    Ok(serde_json::from_str(&s)?)
}

fn run_panel(host: &str, label: &str) -> Run {
    println!("=== probe [{}] -> {} ===", label, host);
    let mut out = Run {
        label: label.into(),
        host: host.into(),
        results: Vec::new(),
    };
    for tc in PANEL {
        let body = json!({
            "start": tc.start,
            "end":   tc.end,
            "modes": MODES,
            "width_m": tc.width_m,
            "alternatives": 1,
            "elevation_samples": ELEV_SAMPLES,
        });
        let t0 = Instant::now();
        let resp = match post(host, "/api/between", body) {
            Ok(r) => r,
            Err(e) => {
                println!("  {}: error {}", tc.name, e);
                continue;
            }
        };
        let elapsed = t0.elapsed().as_secs_f64();
        let results = resp
            .get("results")
            .and_then(|v| v.as_array())
            .cloned()
            .unwrap_or_default();
        if results.is_empty() {
            println!("  {}: no results", tc.name);
            continue;
        }
        let m = metrics_from_result(tc.name, &results[0], elapsed);
        print_row(&m);
        out.results.push(m);
    }
    // Shape-mode regression: fixed seed so the same circle is sampled each
    // run.  This is the regime the user's screenshots exercised — long closed
    // loops that compound any per-edge pathology.
    let body = json!({
        "kind": "circle",
        "perimeter_km": 80.0,
        "modes": MODES,
        "width_m": 4_000,
        "count": 1,
        "seed": 42,
        "elevation_samples": ELEV_SAMPLES,
        "min_viable": 1,
    });
    let t0 = Instant::now();
    match post(host, "/api/shape-search", body) {
        Ok(resp) => {
            let elapsed = t0.elapsed().as_secs_f64();
            let id = resp.get("job_id").and_then(|v| v.as_str()).unwrap_or("");
            let results = resp
                .get("results")
                .and_then(|v| v.as_array())
                .cloned()
                .unwrap_or_default();
            if !results.is_empty() {
                let m = metrics_from_result("shape80", &results[0], elapsed);
                print_row(&m);
                out.results.push(m);
            } else if !id.is_empty() {
                // Result delivered async — poll for completion.
                let mut tries = 0;
                loop {
                    std::thread::sleep(std::time::Duration::from_millis(500));
                    let url = format!("/api/results/{}", id);
                    let r = match get(host, &url) {
                        Ok(r) => r,
                        Err(_) => break,
                    };
                    if r.get("completed").and_then(|v| v.as_bool()).unwrap_or(false) {
                        if let Some(arr) = r.get("results").and_then(|v| v.as_array()) {
                            if let Some(first) = arr.first() {
                                let elapsed_total = t0.elapsed().as_secs_f64();
                                let m = metrics_from_result("shape80", first, elapsed_total);
                                print_row(&m);
                                out.results.push(m);
                            }
                        }
                        break;
                    }
                    tries += 1;
                    if tries > 60 {
                        println!("  shape80: timeout waiting for async results");
                        break;
                    }
                }
            } else {
                println!("  shape80: no results, no job_id");
            }
        }
        Err(e) => println!("  shape80: error {}", e),
    }
    out
}

fn print_row(m: &CaseMetrics) {
    println!(
        "  {:>9}: {:6.1}km det+{:5.1}%  turns {:6.1}°/km  sharp {:5.2}  hp {:4.2}  tails {:>2}  loops {:>2}  surf {:4.2}  asc {:.0}m  ({:.1}s)",
        m.name, m.real_km, m.detour_pct,
        m.turn_density_dps_per_km, m.sharp_turns_per_km, m.hairpins_per_km,
        m.tails, m.detour_loops, m.surface_transitions_per_km,
        m.ascent_m.unwrap_or(0.0), m.elapsed_s,
    );
}

fn get(host: &str, path: &str) -> Result<Value> {
    let url = format!("{}{}", host.trim_end_matches('/'), path);
    let resp = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(60))
        .call()
        .with_context(|| format!("GET {}", url))?;
    Ok(serde_json::from_str(&resp.into_string()?)?)
}

#[derive(Default)]
struct Agg {
    panel: usize,
    detour: f64,
    turn: f64,
    sharp: f64,
    hairpin: f64,
    loops: f64,
    tails: f64,
    surf: f64,
    ascent_pk: f64,
    max_dev: f64,
}

fn aggregate(run: &Run) -> Agg {
    let n = run.results.len() as f64;
    if n == 0.0 {
        return Agg::default();
    }
    let mean = |sel: fn(&CaseMetrics) -> f64| -> f64 {
        run.results.iter().map(sel).sum::<f64>() / n
    };
    Agg {
        panel: run.results.len(),
        detour: mean(|m| m.detour_pct),
        turn: mean(|m| m.turn_density_dps_per_km),
        sharp: mean(|m| m.sharp_turns_per_km),
        hairpin: mean(|m| m.hairpins_per_km),
        loops: mean(|m| m.detour_loops as f64),
        tails: mean(|m| m.tails as f64),
        surf: mean(|m| m.surface_transitions_per_km),
        ascent_pk: mean(|m| m.ascent_per_km.unwrap_or(0.0)),
        max_dev: mean(|m| m.max_dev_m),
    }
}

fn print_summary(run: &Run, compare: Option<&Run>) {
    let agg = aggregate(run);
    if agg.panel == 0 {
        println!("(no results to summarise)");
        return;
    }
    println!("\n=== summary [{}] ===", run.label);
    let cmp = compare.map(aggregate);
    let row = |label: &str, fmt: &str, cur: f64, base: Option<f64>| {
        if let Some(b) = base {
            let delta = cur - b;
            let arrow = if delta < 0.0 { "↓" } else if delta > 0.0 { "↑" } else { "·" };
            match fmt {
                "%.2f" => println!("  {:<24} {:>7.2}  vs {:>7.2}  {}{:.2}", label, cur, b, arrow, delta.abs()),
                "%.1f" => println!("  {:<24} {:>7.1}  vs {:>7.1}  {}{:.1}", label, cur, b, arrow, delta.abs()),
                "%.0f" => println!("  {:<24} {:>7.0}  vs {:>7.0}  {}{:.0}", label, cur, b, arrow, delta.abs()),
                _      => println!("  {:<24} {:>7.2}  vs {:>7.2}  {}{:.2}", label, cur, b, arrow, delta.abs()),
            }
        } else {
            match fmt {
                "%.2f" => println!("  {:<24} {:>7.2}", label, cur),
                "%.1f" => println!("  {:<24} {:>7.1}", label, cur),
                "%.0f" => println!("  {:<24} {:>7.0}", label, cur),
                _      => println!("  {:<24} {:>7.2}", label, cur),
            }
        }
    };
    row("detour %",            "%.2f", agg.detour,    cmp.as_ref().map(|c| c.detour));
    row("turn density °/km",   "%.1f", agg.turn,      cmp.as_ref().map(|c| c.turn));
    row("sharp turns /km",     "%.2f", agg.sharp,     cmp.as_ref().map(|c| c.sharp));
    row("hairpins /km",        "%.2f", agg.hairpin,   cmp.as_ref().map(|c| c.hairpin));
    row("detour loops",        "%.2f", agg.loops,     cmp.as_ref().map(|c| c.loops));
    row("tails",               "%.2f", agg.tails,     cmp.as_ref().map(|c| c.tails));
    row("surf transitions /km","%.2f", agg.surf,      cmp.as_ref().map(|c| c.surf));
    row("ascent m/km",         "%.1f", agg.ascent_pk, cmp.as_ref().map(|c| c.ascent_pk));
    row("max dev m",           "%.0f", agg.max_dev,   cmp.as_ref().map(|c| c.max_dev));
}

/// Quality thresholds the tuned configuration is expected to keep clearing.
/// Pinned in code so a server build that regresses the cost function fails
/// the gate. Numbers come from `experiments/tuned.json` (TURN_PEN=100,
/// compressed class multipliers, hinged ascent, smoothed elevations) with
/// ~15-25 % slack so we don't fail on noise from rebuilds / OSM updates.
struct GateThresholds {
    max_turn_density: f64,    // mean °/km across the between panel
    max_sharp_per_km: f64,    // mean sharp turns (>=60°) per km
    max_hairpins_per_km: f64, // mean hairpins (>=110°) per km
    max_detour_pct: f64,      // mean route detour over straight, %
    max_ascent_per_km: f64,   // mean m/km of climbing
    max_tails: f64,           // mean count of tight self-revisits per route
}

const GATE: GateThresholds = GateThresholds {
    max_turn_density: 260.0, // tuned ~200, +30 % slack
    max_sharp_per_km: 0.75,  // tuned ~0.50
    max_hairpins_per_km: 0.10, // tuned ~0.02
    max_detour_pct: 22.0,    // tuned ~17 %
    max_ascent_per_km: 14.0, // tuned ~11 m/km
    // After loop pruning we expect zero on between-A→B; threshold 0.5 leaves
    // a tiny tolerance for the tail-detector's geometric heuristics on
    // legitimate tight road bends.
    max_tails: 0.5,
};

/// Returns the list of (name, current, threshold) violations or empty.
fn check_gate(run: &Run) -> Vec<(&'static str, f64, f64)> {
    // Gate uses the between-A→B panel only — shape mode has known higher
    // baseline due to fixed polygon vertices and is reported separately.
    let between: Vec<&CaseMetrics> = run.results.iter().filter(|m| m.name != "shape80").collect();
    if between.is_empty() {
        return vec![("panel_empty", 0.0, 0.0)];
    }
    let n = between.len() as f64;
    let mean = |sel: fn(&CaseMetrics) -> f64| -> f64 {
        between.iter().map(|m| sel(*m)).sum::<f64>() / n
    };
    let mut bad = Vec::new();
    let td = mean(|m| m.turn_density_dps_per_km);
    if td > GATE.max_turn_density {
        bad.push(("turn density", td, GATE.max_turn_density));
    }
    let sh = mean(|m| m.sharp_turns_per_km);
    if sh > GATE.max_sharp_per_km {
        bad.push(("sharp turns/km", sh, GATE.max_sharp_per_km));
    }
    let hp = mean(|m| m.hairpins_per_km);
    if hp > GATE.max_hairpins_per_km {
        bad.push(("hairpins/km", hp, GATE.max_hairpins_per_km));
    }
    let dp = mean(|m| m.detour_pct);
    if dp > GATE.max_detour_pct {
        bad.push(("detour %", dp, GATE.max_detour_pct));
    }
    let ap = mean(|m| m.ascent_per_km.unwrap_or(0.0));
    if ap > GATE.max_ascent_per_km {
        bad.push(("ascent m/km", ap, GATE.max_ascent_per_km));
    }
    let tl = mean(|m| m.tails as f64);
    if tl > GATE.max_tails {
        bad.push(("tails / route", tl, GATE.max_tails));
    }
    bad
}

#[derive(Default)]
struct Cli {
    host: String,
    label: Option<String>,
    compare: Option<String>,
    gate: bool,
}

fn parse_args() -> Cli {
    let mut cli = Cli {
        host: "http://127.0.0.1:7878".to_string(),
        ..Default::default()
    };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--host" => {
                cli.host = args.get(i + 1).cloned().unwrap_or_default();
                i += 2;
            }
            "--label" => {
                cli.label = args.get(i + 1).cloned();
                i += 2;
            }
            "--compare" => {
                cli.compare = args.get(i + 1).cloned();
                i += 2;
            }
            "--gate" => {
                cli.gate = true;
                i += 1;
            }
            _ => {
                i += 1;
            }
        }
    }
    cli
}

fn main() -> Result<()> {
    let cli = parse_args();
    let label = cli.label.clone().unwrap_or_else(|| {
        if cli.gate {
            "gate".to_string()
        } else {
            eprintln!("error: --label is required (or pass --gate)");
            std::process::exit(2);
        }
    });
    let runs_dir = PathBuf::from("experiments");
    fs::create_dir_all(&runs_dir).ok();
    let run = run_panel(&cli.host, &label);
    let path = runs_dir.join(format!("{}.json", label));
    fs::write(&path, serde_json::to_string_pretty(&run)?)?;
    println!("wrote {}", path.display());
    let compare_run: Option<Run> = if let Some(c) = cli.compare {
        let cp = runs_dir.join(format!("{}.json", c));
        if cp.exists() {
            Some(serde_json::from_str(&fs::read_to_string(&cp)?)?)
        } else {
            println!("(compare file {} not found)", cp.display());
            None
        }
    } else {
        None
    };
    print_summary(&run, compare_run.as_ref());
    if cli.gate {
        let violations = check_gate(&run);
        if violations.is_empty() {
            println!("\n=== gate: PASS ({} thresholds) ===", 6);
            Ok(())
        } else {
            eprintln!("\n=== gate: FAIL ===");
            for (name, cur, thr) in &violations {
                eprintln!("  {:<20} current {:>7.2}  >  threshold {:>7.2}", name, cur, thr);
            }
            std::process::exit(1);
        }
    } else {
        Ok(())
    }
}
