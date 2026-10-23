//! End-to-end checks on a real OSM extract (Liechtenstein, ~3 MB): build the
//! graph, serve the HTTP API on an ephemeral port, and exercise every
//! endpoint, plus a graph-cache round trip on real data.
//!
//! Skipped unless the extract is available. CI downloads it and sets
//! `CROWFLY_E2E_PBF`, plus `CROWFLY_E2E_REQUIRED=1` so a missing file fails
//! instead of skipping; locally `osm-cache/liechtenstein-latest.osm.pbf` is
//! used when present. OSM data changes daily, so the assertions check
//! invariants (bounds, determinism, response shape), not exact routes.

use crowfly::geodesy::{haversine, LatLon};
use crowfly::graph_cache;
use crowfly::osm::{Graph, MODE_BIKE, MODE_FOOT, MODE_MTB};
use crowfly::route::{self, RouteParams};
use crowfly::server::{router, AppState, ServerState};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

const MODES: u8 = MODE_FOOT | MODE_BIKE | MODE_MTB;
/// [lat, lon]: Vaduz (valley, ~455 m) and Malbun (~1600 m), ~9 km apart.
const VADUZ: [f64; 2] = [47.1410, 9.5209];
const MALBUN: [f64; 2] = [47.1017, 9.6097];

fn extract() -> Option<PathBuf> {
    let path = std::env::var_os("CROWFLY_E2E_PBF")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("osm-cache/liechtenstein-latest.osm.pbf"));
    if path.exists() {
        return Some(path);
    }
    assert!(
        std::env::var_os("CROWFLY_E2E_REQUIRED").is_none(),
        "CROWFLY_E2E_REQUIRED is set but {} is missing",
        path.display()
    );
    eprintln!(
        "skipping: {} not found (set CROWFLY_E2E_PBF)",
        path.display()
    );
    None
}

/// Serve `graph` (no DEM, no no-go zones) on an ephemeral port from a
/// background runtime and return the base URL once it answers.
fn spawn_server(graph: Graph) -> String {
    let state = AppState(Arc::new(ServerState::new(graph, Vec::new(), 800.0, None)));
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    listener.set_nonblocking(true).unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let listener = tokio::net::TcpListener::from_std(listener).unwrap();
            axum::serve(listener, router(state)).await.unwrap();
        });
    });
    let deadline = Instant::now() + Duration::from_secs(10);
    while get(&base, "/api/info").0 != 200 {
        assert!(Instant::now() < deadline, "server did not come up");
        std::thread::sleep(Duration::from_millis(50));
    }
    base
}

fn parse(body: String) -> Value {
    serde_json::from_str(&body).unwrap_or(Value::Null)
}

fn get(base: &str, path: &str) -> (u16, Value) {
    match ureq::get(&format!("{base}{path}")).call() {
        Ok(r) => (r.status(), parse(r.into_string().unwrap_or_default())),
        Err(ureq::Error::Status(code, r)) => (code, parse(r.into_string().unwrap_or_default())),
        Err(_) => (0, Value::Null),
    }
}

fn post(base: &str, path: &str, body: &Value) -> (u16, Value) {
    let req = ureq::post(&format!("{base}{path}"))
        .timeout(Duration::from_secs(300))
        .set("Content-Type", "application/json");
    match req.send_string(&body.to_string()) {
        Ok(r) => (r.status(), parse(r.into_string().unwrap_or_default())),
        Err(ureq::Error::Status(code, r)) => (code, parse(r.into_string().unwrap_or_default())),
        Err(e) => panic!("POST {path}: {e}"),
    }
}

fn results(j: &Value) -> &Vec<Value> {
    j["results"].as_array().expect("results array")
}

fn lonlat(v: &Value) -> LatLon {
    LatLon::new(v[1].as_f64().unwrap(), v[0].as_f64().unwrap())
}

/// Angle (degrees, 0–180) between `a` and `b` as seen from `centre`.
fn bearing_gap(centre: LatLon, a: LatLon, b: LatLon) -> f64 {
    let bearing = |p: LatLon| {
        let east = (p.lon - centre.lon) * centre.lat.to_radians().cos();
        east.atan2(p.lat - centre.lat).to_degrees()
    };
    let gap = (bearing(a) - bearing(b)).abs() % 360.0;
    gap.min(360.0 - gap)
}

#[test]
fn api_end_to_end() {
    let Some(pbf) = extract() else { return };
    let graph = graph_cache::build_or_load(&pbf, MODES, None).unwrap();
    let base = spawn_server(graph);

    for page in ["/", "/app.js", "/style.css"] {
        assert_eq!(get(&base, page).0, 200, "GET {page}");
    }
    let (_, info) = get(&base, "/api/info");
    assert!(info["graph_nodes"].as_u64().unwrap() > 50_000, "{info}");
    let b: Vec<f64> = info["bbox"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    assert!(
        b[0] > 9.4 && b[1] > 47.0 && b[2] < 9.7 && b[3] < 47.3,
        "bbox {b:?}"
    );

    // Between: a real route inside the corridor, surfaces adding up.
    let between = json!({ "start": VADUZ, "end": MALBUN, "modes": "foot,bike",
                          "width_m": 4000, "elevation_samples": 30 });
    let (status, j) = post(&base, "/api/between", &between);
    assert_eq!(status, 200, "{j}");
    assert!(!results(&j).is_empty());
    for r in results(&j) {
        let (ref_km, real_km) = (
            r["ref_km"].as_f64().unwrap(),
            r["real_km"].as_f64().unwrap(),
        );
        assert!(
            real_km > 0.8 * ref_km && real_km < 3.0 * ref_km,
            "{ref_km} -> {real_km}"
        );
        assert!(r["max_dev_m"].as_f64().unwrap() <= 2000.0 + 1.0);
        let surface: f64 = r["surface"]
            .as_array()
            .unwrap()
            .iter()
            .map(|s| s["km"].as_f64().unwrap())
            .sum();
        assert!(
            (surface - real_km).abs() < 0.02 * real_km,
            "{surface} vs {real_km}"
        );
        assert!(r["coords"].as_array().unwrap().len() >= 2);
        assert!(r["elevation"].is_null(), "no DEM loaded, so no profile");
    }
    let mut bad = between.clone();
    bad["modes"] = json!("rocket");
    let (status, j) = post(&base, "/api/between", &bad);
    assert_eq!(status, 400, "bad mode is a client error: {j}");

    // Country: bounds respected, seed echoed, same seed = same results.
    let country = json!({ "modes": "foot,bike,mtb", "elevation_samples": 0, "width_m": 3000,
                          "count": 4, "distance_min_km": 8, "distance_max_km": 20, "seed": 7 });
    let (status, a) = post(&base, "/api/country", &country);
    assert_eq!(status, 200, "{a}");
    assert_eq!(a["seed"], 7);
    assert!(results(&a).len() <= 8, "at most `count` per section");
    for r in results(&a) {
        let ref_km = r["ref_km"].as_f64().unwrap();
        let real_km = r["real_km"].as_f64().unwrap();
        assert!((8.0..=20.0).contains(&ref_km) && (8.0..=20.0).contains(&real_km));
        let half = r["corridor_used_m"].as_f64().unwrap() / 2.0;
        assert!(r["max_dev_m"].as_f64().unwrap() <= half + 1.0);
    }
    let (_, again) = post(&base, "/api/country", &country);
    assert_eq!(
        a["results"], again["results"],
        "seeded search must replay exactly"
    );

    // Crossings: no length bounds needed; both ends on the country's
    // outline, on roughly opposite sides of its centre.
    let crossing = json!({ "modes": "foot,bike,mtb", "elevation_samples": 0, "width_m": 3000,
                           "border_to_border": true, "seed": 3 });
    let (status, j) = post(&base, "/api/country", &crossing);
    assert_eq!(status, 200, "{j}");
    assert!(!results(&j).is_empty(), "no crossing found: {j}");
    let centre = lonlat(&info["center"]);
    for r in results(&j) {
        let gap = bearing_gap(centre, lonlat(&r["start"]), lonlat(&r["end"]));
        assert!(
            gap >= 119.0,
            "crossing ends only {gap:.0}° apart around the centre"
        );
    }

    // Shape: closed loops.
    let shape = json!({ "kind": "circle", "perimeter_km": 10, "width_m": 5000, "count": 2,
                        "modes": "foot,bike,mtb", "seed": 11, "elevation_samples": 0 });
    let (status, j) = post(&base, "/api/shape-search", &shape);
    assert_eq!(status, 200, "{j}");
    assert_eq!(j["seed"], 11);
    for r in results(&j) {
        assert_eq!(r["shape"]["kind"], "circle");
        let c = r["coords"].as_array().unwrap();
        let gap = haversine(lonlat(&c[0]), lonlat(&c[c.len() - 1]));
        assert!(gap < 1000.0, "loop not closed: {gap:.0} m");
    }

    // Text.
    let text = json!({ "text": "LI", "letter_height_m": 4000, "candidates": 60, "seed": 7,
                       "modes": "foot,bike,mtb", "elevation_samples": 0 });
    let (status, j) = post(&base, "/api/text", &text);
    assert_eq!(status, 200, "{j}");
    assert_eq!(j["seed"], 7);
    if let Some(r) = results(&j).first() {
        assert_eq!(r["kind"], "text");
        assert!(!r["coords"].as_array().unwrap().is_empty());
    }

    // GPX elevations: null without a DEM.
    let (status, j) = post(
        &base,
        "/api/elevations",
        &json!({ "coords": [[9.52, 47.14]] }),
    );
    assert_eq!((status, j), (200, json!({ "ele": null })));

    // Cancel: a long search answers 409 soon after /api/cancel...
    let slow = json!({ "modes": "foot,bike,mtb", "elevation_samples": 0, "width_m": 4000,
                       "count": 50, "distance_min_km": 5, "distance_max_km": 25,
                       "min_viable": 100000, "seed": 1 });
    let b2 = base.clone();
    let search = std::thread::spawn(move || {
        let t = Instant::now();
        (post(&b2, "/api/country", &slow).0, t.elapsed())
    });
    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(post(&base, "/api/cancel", &json!({})).0, 204);
    let (status, took) = search.join().unwrap();
    assert_eq!(status, 409);
    assert!(took < Duration::from_secs(15), "cancel took {took:?}");
    // ...and the server is fine afterwards.
    assert_eq!(post(&base, "/api/between", &between).0, 200);
}

#[test]
fn graph_cache_round_trip_on_real_data() {
    let Some(pbf) = extract() else { return };
    let dir = std::env::temp_dir().join(format!("crowfly-e2e-cache-{}", std::process::id()));
    let fresh = graph_cache::build_or_load(&pbf, MODES, Some(&dir)).unwrap();
    let key = graph_cache::cache_key(&pbf, MODES).unwrap();
    let cached = graph_cache::load(&graph_cache::cache_path(&dir, &pbf), key)
        .unwrap()
        .expect("build_or_load wrote a cache with the current key");
    let _ = std::fs::remove_dir_all(&dir);

    assert_eq!(cached.graph.node_count(), fresh.graph.node_count());
    assert_eq!(cached.graph.edge_count(), fresh.graph.edge_count());
    let params = RouteParams {
        modes: MODE_FOOT | MODE_BIKE,
        half_width_m: 2000.0,
        alpha: 4.0,
        corridor_max_m: 2000.0,
        paved_only: false,
    };
    let (a, b) = (
        LatLon::new(VADUZ[0], VADUZ[1]),
        LatLon::new(MALBUN[0], MALBUN[1]),
    );
    let route_on = |g: &Graph| {
        let (s, e, _, _) = g.closest_connected_pair(a, b).unwrap();
        route::shortest(g, s, e, a, b, &params).unwrap().points
    };
    assert_eq!(
        route_on(&cached),
        route_on(&fresh),
        "cached graph routes identically"
    );
}
