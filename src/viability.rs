//! Post-routing viability checks: no-go intersections, mode breakdown,
//! ferry/lake segments, max corridor deviation.

use crate::geodesy::{cross_track, haversine, LatLon};
use crate::osm::{MODE_BIKE, MODE_FERRY, MODE_FOOT, MODE_MTB, SURFACE_LABELS};
use crate::route::RouteResult;
use anyhow::{Context, Result};
use geo::{Coord, Intersects, LineString, MultiPolygon, Polygon};
use serde_json::Value;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct NoGoZone {
    pub name: String,
    pub polygons: MultiPolygon<f64>,
}

#[derive(Debug, Clone)]
pub struct Report {
    pub total_km: f64,
    pub max_deviation_m: f64,
    pub avg_deviation_m: f64,
    pub foot_km: f64,
    pub bike_km: f64,
    pub mtb_km: f64,
    pub ferry_km: f64,
    /// Length km per surface category, indexed by SURFACE_* constants.
    pub surface_km: [f64; 6],
    pub violations: Vec<NoGoViolation>,
    pub long_edges: Vec<LongEdge>,
}

#[derive(Debug, Clone)]
pub struct NoGoViolation {
    pub zone: String,
}

#[derive(Debug, Clone)]
pub struct LongEdge {
    pub from: LatLon,
    pub to: LatLon,
    pub length_m: f64,
    pub is_ferry: bool,
}

pub fn load_no_go(path: &Path) -> Result<NoGoZone> {
    let s = std::fs::read_to_string(path)
        .with_context(|| format!("reading no-go file {}", path.display()))?;
    let v: Value = serde_json::from_str(&s)?;
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("nogo")
        .to_string();

    let mut polys: Vec<Polygon<f64>> = Vec::new();
    extract_polygons(&v, &mut polys);
    Ok(NoGoZone {
        name,
        polygons: MultiPolygon(polys),
    })
}

fn extract_polygons(v: &Value, out: &mut Vec<Polygon<f64>>) {
    match v {
        Value::Object(map) => {
            if let Some(t) = map.get("type").and_then(|s| s.as_str()) {
                match t {
                    "FeatureCollection" => {
                        if let Some(arr) = map.get("features").and_then(|f| f.as_array()) {
                            for f in arr {
                                extract_polygons(f, out);
                            }
                        }
                    }
                    "Feature" => {
                        if let Some(g) = map.get("geometry") {
                            extract_polygons(g, out);
                        }
                    }
                    "Polygon" => {
                        if let Some(coords) = map.get("coordinates") {
                            if let Some(p) = parse_polygon(coords) {
                                out.push(p);
                            }
                        }
                    }
                    "MultiPolygon" => {
                        if let Some(arr) = map.get("coordinates").and_then(|c| c.as_array()) {
                            for poly in arr {
                                if let Some(p) = parse_polygon(poly) {
                                    out.push(p);
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        _ => {}
    }
}

fn parse_polygon(v: &Value) -> Option<Polygon<f64>> {
    let rings = v.as_array()?;
    let mut iter = rings.iter();
    let outer = parse_ring(iter.next()?)?;
    let holes: Vec<LineString<f64>> = iter.filter_map(parse_ring).collect();
    Some(Polygon::new(outer, holes))
}

fn parse_ring(v: &Value) -> Option<LineString<f64>> {
    let arr = v.as_array()?;
    let coords: Vec<Coord<f64>> = arr
        .iter()
        .filter_map(|p| {
            let pa = p.as_array()?;
            let lon = pa.first()?.as_f64()?;
            let lat = pa.get(1)?.as_f64()?;
            Some(Coord { x: lon, y: lat })
        })
        .collect();
    if coords.len() < 3 {
        None
    } else {
        Some(LineString::from(coords))
    }
}

pub fn analyse(
    route: &RouteResult,
    a: LatLon,
    b: LatLon,
    no_go: &[NoGoZone],
    long_edge_threshold_m: f64,
) -> Report {
    let mut foot = 0.0;
    let mut bike = 0.0;
    let mut mtb = 0.0;
    let mut ferry = 0.0;
    let mut total = 0.0;
    let mut sum_dev = 0.0;
    let mut max_dev = 0.0f64;
    let mut surface_km = [0.0f64; 6];
    let mut long_edges: Vec<LongEdge> = Vec::new();

    for (i, e) in route.edges.iter().enumerate() {
        total += e.length_m;
        let p0 = route.points[i];
        let p1 = route.points[i + 1];
        let mid = LatLon::new((p0.lat + p1.lat) * 0.5, (p0.lon + p1.lon) * 0.5);
        let dev = cross_track(mid, a, b).abs();
        sum_dev += dev * e.length_m;
        if dev > max_dev {
            max_dev = dev;
        }

        if (e.modes & MODE_FERRY) != 0 {
            ferry += e.length_m;
        } else if (e.modes & MODE_FOOT) != 0 {
            foot += e.length_m;
        } else if (e.modes & MODE_BIKE) != 0 {
            bike += e.length_m;
        } else if (e.modes & MODE_MTB) != 0 {
            mtb += e.length_m;
        }
        let s_idx = (e.surface as usize).min(5);
        surface_km[s_idx] += e.length_m / 1000.0;

        if e.length_m >= long_edge_threshold_m {
            long_edges.push(LongEdge {
                from: p0,
                to: p1,
                length_m: e.length_m,
                is_ferry: (e.modes & MODE_FERRY) != 0,
            });
        }
    }

    // Per-vertex check catches mid-segment excursions if the route deviates
    // wildly between two graph nodes.
    for p in &route.points {
        let dev = cross_track(*p, a, b).abs();
        if dev > max_dev {
            max_dev = dev;
        }
    }

    let avg_dev = if total > 0.0 { sum_dev / total } else { 0.0 };

    // No-go intersections.
    let route_ls: LineString<f64> = LineString::from(
        route
            .points
            .iter()
            .map(|p| Coord {
                x: p.lon,
                y: p.lat,
            })
            .collect::<Vec<_>>(),
    );
    let mut violations = Vec::new();
    for z in no_go {
        if z.polygons.intersects(&route_ls) {
            violations.push(NoGoViolation {
                zone: z.name.clone(),
            });
        }
    }

    Report {
        total_km: total / 1000.0,
        max_deviation_m: max_dev,
        avg_deviation_m: avg_dev,
        foot_km: foot / 1000.0,
        bike_km: bike / 1000.0,
        mtb_km: mtb / 1000.0,
        ferry_km: ferry / 1000.0,
        surface_km,
        violations,
        long_edges,
    }
}

/// Pretty-print summary.
pub fn print_report(report: &Report, ref_distance_km: f64) {
    println!("\n— Route summary —");
    println!(
        "Reference (straight) distance : {:>8.2} km",
        ref_distance_km
    );
    println!(
        "Routed distance               : {:>8.2} km   (+{:.1}% over straight)",
        report.total_km,
        100.0 * (report.total_km / ref_distance_km - 1.0).max(0.0)
    );
    println!(
        "Max corridor deviation        : {:>8.0} m",
        report.max_deviation_m
    );
    println!(
        "Avg corridor deviation        : {:>8.0} m",
        report.avg_deviation_m
    );
    println!("\nMode breakdown:");
    println!("  foot     : {:>7.2} km", report.foot_km);
    println!("  bike     : {:>7.2} km", report.bike_km);
    println!("  mtb/path : {:>7.2} km", report.mtb_km);
    println!("  ferry    : {:>7.2} km", report.ferry_km);
    println!("\nSurface breakdown:");
    for (i, label) in SURFACE_LABELS.iter().enumerate() {
        if report.surface_km[i] > 0.05 {
            println!("  {:<8} : {:>7.2} km", label, report.surface_km[i]);
        }
    }

    if !report.long_edges.is_empty() {
        println!("\nLong single edges (likely ferries / disconnected):");
        for le in &report.long_edges {
            println!(
                "  {:>6.2} km  ferry={}  ({:.4},{:.4}) → ({:.4},{:.4})",
                le.length_m / 1000.0,
                le.is_ferry,
                le.from.lat,
                le.from.lon,
                le.to.lat,
                le.to.lon
            );
        }
    }

    if report.violations.is_empty() {
        println!("\nNo-go zones: no intersections.");
    } else {
        println!("\nNO-GO ZONE VIOLATIONS:");
        for v in &report.violations {
            println!("  ✗ route enters: {}", v.zone);
        }
    }
}

/// Avoid unused-import warnings when we want one helper visible.
#[allow(dead_code)]
pub fn straight_distance_km(a: LatLon, b: LatLon) -> f64 {
    haversine(a, b) / 1000.0
}
