//! Write GPX, GeoJSON FeatureCollection, and a self-contained HTML viewer.

use crate::elevation::Profile;
use crate::explore::Scored;
use crate::geodesy::{corridor_polygon, haversine, sample_line, LatLon};
use crate::osm::SURFACE_LABELS;
use crate::route::RouteResult;
use crate::viability::{NoGoZone, Report};
use anyhow::Result;
use serde_json::{json, Value};
use std::fs;
use std::path::Path;

pub struct Outputs<'a> {
    pub start: LatLon,
    pub end: LatLon,
    pub corridor_width_m: f64,
    pub route: &'a RouteResult,
    pub no_go: &'a [NoGoZone],
    pub report: &'a Report,
    pub elevation: Option<&'a Profile>,
}

impl<'a> Outputs<'a> {
    pub fn write_all(&self, out_dir: &Path) -> Result<()> {
        fs::create_dir_all(out_dir)?;
        let geojson = self.geojson();
        fs::write(
            out_dir.join("route.geojson"),
            serde_json::to_string_pretty(&geojson)?,
        )?;
        fs::write(out_dir.join("route.gpx"), self.gpx())?;
        fs::write(out_dir.join("viewer.html"), self.html(&geojson))?;
        Ok(())
    }

    fn geojson(&self) -> Value {
        let mut features = Vec::new();

        // Reference line, sampled densely so MapLibre draws a smooth great circle.
        let ref_pts = sample_line(self.start, self.end, 2_000.0);
        features.push(linestring_feature(
            "reference",
            "Reference straight line",
            &ref_pts,
        ));

        // Corridor polygon.
        let corridor = corridor_polygon(self.start, self.end, self.corridor_width_m);
        features.push(polygon_feature("corridor", "Corridor", &corridor));

        // Routed path.
        features.push(linestring_feature(
            "route",
            "Routed path",
            &self.route.points,
        ));

        // No-go zones.
        for z in self.no_go {
            for poly in &z.polygons.0 {
                let outer: Vec<LatLon> = poly
                    .exterior()
                    .points()
                    .map(|p| LatLon::new(p.y(), p.x()))
                    .collect();
                features.push(polygon_feature("no-go", &z.name, &outer));
            }
        }

        // Endpoints.
        features.push(point_feature("endpoint", "Start", self.start));
        features.push(point_feature("endpoint", "End", self.end));

        // Long edges (ferry / disconnected) as visible warnings.
        for le in &self.report.long_edges {
            features.push(linestring_feature(
                "long-edge",
                &format!(
                    "{} {:.1} km",
                    if le.is_ferry { "ferry" } else { "long edge" },
                    le.length_m / 1000.0
                ),
                &[le.from, le.to],
            ));
        }

        json!({
            "type": "FeatureCollection",
            "features": features,
        })
    }

    pub fn gpx(&self) -> String {
        let mut s = String::new();
        s.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
        s.push('\n');
        s.push_str(
            r#"<gpx version="1.1" creator="crowfly" xmlns="http://www.topografix.com/GPX/1/1">"#,
        );
        s.push('\n');
        s.push_str("  <metadata><name>Straight-line crossing</name></metadata>\n");
        s.push_str("  <trk><name>Routed path</name><trkseg>\n");
        for p in &self.route.points {
            s.push_str(&format!(
                "    <trkpt lat=\"{:.7}\" lon=\"{:.7}\"/>\n",
                p.lat, p.lon
            ));
        }
        s.push_str("  </trkseg></trk>\n");
        s.push_str("</gpx>\n");
        s
    }

    fn html(&self, geojson: &Value) -> String {
        let r = self.report;
        let detour = if !r.total_km.is_nan() && self.route.points.len() >= 2 {
            let ref_km = haversine(self.start, self.end) / 1000.0;
            if ref_km > 0.0 {
                100.0 * (r.total_km / ref_km - 1.0)
            } else {
                0.0
            }
        } else {
            0.0
        };
        let elev = match self.elevation {
            Some(p) => format!(
                "<div class=\"elev-row\"><span>↗ {:.0} m</span><span>↘ {:.0} m</span>\
                 <span>max {:.0} m</span><span>min {:.0} m</span></div>{}",
                p.ascent_m,
                p.descent_m,
                p.max_m,
                p.min_m,
                elevation_svg(p, 480, 60)
            ),
            None => String::new(),
        };
        let surface = surface_bar_html(r);
        let viol = if r.violations.is_empty() {
            "<span style='color:#0a0'>no no-go intersections</span>".to_string()
        } else {
            format!(
                "<span style='color:#a00'>VIOLATIONS: {}</span>",
                r.violations
                    .iter()
                    .map(|v| v.zone.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        };
        let stats = format!(
            "<div class=\"stats\"><b>Routed:</b> {:.1} km \
             ({:+.0}% over straight) &middot; \
             <b>Max dev:</b> {:.0} m &middot; \
             <b>Ferry:</b> {:.1} km</div>{surface}{elev}<div class=\"viol\">{viol}</div>",
            r.total_km, detour, r.max_deviation_m, r.ferry_km
        );

        let geojson_str = serde_json::to_string(geojson).unwrap();

        HTML_TEMPLATE
            .replace("/*GEOJSON*/", &geojson_str)
            .replace("/*STATS*/", &serde_json::to_string(&stats).unwrap())
            .replace("/*SWLEGEND*/", &surface_legend_html())
    }
}

fn linestring_feature(kind: &str, name: &str, pts: &[LatLon]) -> Value {
    let coords: Vec<Vec<f64>> = pts.iter().map(|p| vec![p.lon, p.lat]).collect();
    json!({
        "type": "Feature",
        "properties": {"kind": kind, "name": name},
        "geometry": {"type": "LineString", "coordinates": coords},
    })
}

fn polygon_feature(kind: &str, name: &str, ring: &[LatLon]) -> Value {
    let coords: Vec<Vec<f64>> = ring.iter().map(|p| vec![p.lon, p.lat]).collect();
    json!({
        "type": "Feature",
        "properties": {"kind": kind, "name": name},
        "geometry": {"type": "Polygon", "coordinates": [coords]},
    })
}

fn point_feature(kind: &str, name: &str, p: LatLon) -> Value {
    json!({
        "type": "Feature",
        "properties": {"kind": kind, "name": name},
        "geometry": {"type": "Point", "coordinates": [p.lon, p.lat]},
    })
}

/// Output for explore mode: many candidate lines overlaid, ranked by score.
pub struct ExploreOutputs<'a> {
    pub start: LatLon,
    pub corridor_width_m: f64,
    pub ranked: &'a [Scored],
    pub no_go: &'a [NoGoZone],
}

impl<'a> ExploreOutputs<'a> {
    pub fn write_all(&self, out_dir: &Path) -> Result<()> {
        fs::create_dir_all(out_dir)?;
        let geojson = self.geojson();
        fs::write(
            out_dir.join("explore.geojson"),
            serde_json::to_string_pretty(&geojson)?,
        )?;
        fs::write(out_dir.join("explore.html"), self.html(&geojson))?;
        // Also dump the top route as a stand-alone GPX for convenience.
        if let Some(top) = self.ranked.first() {
            let single = Outputs {
                start: self.start,
                end: top.candidate.end,
                corridor_width_m: self.corridor_width_m,
                route: &top.route,
                no_go: self.no_go,
                report: &top.report,
                elevation: top.elevation.as_ref(),
            };
            fs::write(out_dir.join("top1.gpx"), single.gpx())?;
        }
        Ok(())
    }

    fn geojson(&self) -> Value {
        let mut features = Vec::new();
        // Start marker.
        features.push(point_feature("endpoint", "Start", self.start));

        for (i, s) in self.ranked.iter().enumerate() {
            let rank = i + 1;
            let line = format!("rank {} ({:.0}°, {:.1} km)", rank, s.candidate.bearing_deg, s.report.total_km);
            // Reference (great-circle) line dashed, dimmed.
            let ref_pts = sample_line(self.start, s.candidate.end, 3_000.0);
            features.push(linestring_feature_props(
                "explore-reference",
                &line,
                &ref_pts,
                json!({"rank": rank, "score": s.score}),
            ));
            // Routed path.
            features.push(linestring_feature_props(
                "explore-route",
                &line,
                &s.route.points,
                json!({
                    "rank": rank,
                    "score": s.score,
                    "ref_km": (haversine_helper(self.start, s.candidate.end) / 1000.0),
                    "real_km": s.report.total_km,
                    "max_dev_m": s.report.max_deviation_m,
                    "ferry_km": s.report.ferry_km,
                    "bearing": s.candidate.bearing_deg,
                }),
            ));
            // Endpoint dot.
            features.push(point_feature_props(
                "explore-end",
                &line,
                s.candidate.end,
                json!({"rank": rank, "score": s.score}),
            ));
        }

        // No-go zones — shared across all candidates.
        for z in self.no_go {
            for poly in &z.polygons.0 {
                let outer: Vec<LatLon> = poly
                    .exterior()
                    .points()
                    .map(|p| LatLon::new(p.y(), p.x()))
                    .collect();
                features.push(polygon_feature("no-go", &z.name, &outer));
            }
        }

        json!({"type": "FeatureCollection", "features": features})
    }

    fn html(&self, geojson: &Value) -> String {
        let mut cards = String::new();
        for (i, s) in self.ranked.iter().enumerate() {
            let ref_km = haversine_helper(self.start, s.candidate.end) / 1000.0;
            cards.push_str(&card_html(i + 1, ref_km, s));
        }
        EXPLORE_HTML_TEMPLATE
            .replace("/*GEOJSON*/", &serde_json::to_string(geojson).unwrap())
            .replace("/*CARDS*/", &serde_json::to_string(&cards).unwrap())
            .replace("/*SWLEGEND*/", &surface_legend_html())
    }
}

fn haversine_helper(a: LatLon, b: LatLon) -> f64 {
    crate::geodesy::haversine(a, b)
}

const SURFACE_COLORS: [&str; 6] = [
    "#444444", // paved
    "#c9a64f", // gravel
    "#6aa84f", // path
    "#8b4513", // unpaved
    "#888888", // other
    "#2a8fe8", // ferry
];

fn surface_bar_html(report: &Report) -> String {
    let total: f64 = report.surface_km.iter().sum();
    if total <= 0.0 {
        return String::new();
    }
    let mut segs = String::new();
    let mut tooltip = String::new();
    for (i, &km) in report.surface_km.iter().enumerate() {
        if km <= 0.001 {
            continue;
        }
        let pct = 100.0 * km / total;
        segs.push_str(&format!(
            "<span class=\"seg\" style=\"width:{:.2}%;background:{}\" title=\"{} {:.1} km\"></span>",
            pct, SURFACE_COLORS[i], SURFACE_LABELS[i], km
        ));
        if !tooltip.is_empty() {
            tooltip.push_str(" · ");
        }
        tooltip.push_str(&format!("{} {:.1}", SURFACE_LABELS[i], km));
    }
    format!(
        "<div class=\"bar-wrap\"><div class=\"bar\" title=\"{}\">{}</div></div>",
        tooltip, segs
    )
}

fn surface_legend_html() -> String {
    let mut s = String::from("<div class=\"swlegend\">");
    for (i, label) in SURFACE_LABELS.iter().enumerate() {
        s.push_str(&format!(
            "<span><span class=\"sw\" style=\"background:{}\"></span>{}</span>",
            SURFACE_COLORS[i], label
        ));
    }
    s.push_str("</div>");
    s
}

fn elevation_svg(profile: &Profile, w: u32, h: u32) -> String {
    let n = profile.samples_m.len();
    if n < 2 {
        return String::new();
    }
    let span = (profile.max_m - profile.min_m).max(1.0);
    let mut pts = String::with_capacity(n * 12);
    let mut area = String::with_capacity(n * 12 + 32);
    let xs: Vec<f64> = (0..n)
        .map(|i| (i as f64) / ((n - 1) as f64) * (w as f64))
        .collect();
    let ys: Vec<f64> = profile
        .samples_m
        .iter()
        .map(|&e| {
            let t = (e - profile.min_m) / span;
            (h as f64) - t * (h as f64 - 4.0) - 2.0
        })
        .collect();
    for i in 0..n {
        if i > 0 {
            pts.push(' ');
        }
        pts.push_str(&format!("{:.1},{:.1}", xs[i], ys[i]));
    }
    // Closed area path.
    area.push_str(&format!("M{:.1},{} ", xs[0], h));
    for i in 0..n {
        area.push_str(&format!("L{:.1},{:.1} ", xs[i], ys[i]));
    }
    area.push_str(&format!("L{:.1},{} Z", xs[n - 1], h));
    format!(
        "<svg class=\"spark\" viewBox=\"0 0 {w} {h}\" preserveAspectRatio=\"none\" \
         xmlns=\"http://www.w3.org/2000/svg\">\
         <path d=\"{area}\" fill=\"rgba(230,57,70,0.18)\" stroke=\"none\"/>\
         <polyline points=\"{pts}\" fill=\"none\" stroke=\"#e63946\" stroke-width=\"1.6\"/>\
         </svg>"
    )
}

fn card_html(rank: usize, ref_km: f64, s: &Scored) -> String {
    let r = &s.report;
    let detour = if ref_km > 0.0 {
        100.0 * (r.total_km / ref_km - 1.0)
    } else {
        0.0
    };
    let elev = match &s.elevation {
        Some(p) => format!(
            "<div class=\"elev-row\"><span>↗ {:.0} m</span><span>↘ {:.0} m</span>\
             <span>max {:.0} m</span><span>min {:.0} m</span></div>{}",
            p.ascent_m,
            p.descent_m,
            p.max_m,
            p.min_m,
            elevation_svg(p, 320, 50)
        ),
        None => String::new(),
    };
    let surface = surface_bar_html(r);
    format!(
        "<div class=\"card\" data-rank=\"{rank}\">\
           <div class=\"hd\"><b>#{rank}</b> &nbsp; {bearing:.0}° &nbsp; \
              <span class=\"score\">score {score:.1}</span></div>\
           <div class=\"row\">{ref_km:.1} → <b>{real_km:.1} km</b> ({detour:+.0}%) · \
              max dev {dev:.0} m{ferry}</div>\
           {surface}\
           {elev}\
           <div class=\"actions\"><button class=\"view\">view</button>\
              <button class=\"dl\">⬇ GPX</button></div>\
         </div>",
        rank = rank,
        bearing = s.candidate.bearing_deg,
        score = s.score,
        ref_km = ref_km,
        real_km = r.total_km,
        detour = detour,
        dev = r.max_deviation_m,
        ferry = if r.ferry_km > 0.05 {
            format!(" · ferry {:.1} km", r.ferry_km)
        } else {
            String::new()
        },
        surface = surface,
        elev = elev,
    )
}

const EXPLORE_HTML_TEMPLATE: &str = r#"<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>Straight-line explore</title>
<link href="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.css" rel="stylesheet" />
<style>
  body { margin: 0; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
  #map { position: absolute; top: 0; bottom: 0; left: 0; right: 380px; }
  #side {
    position: absolute; top: 0; bottom: 0; right: 0; width: 380px;
    background: #fafafa; box-sizing: border-box; overflow: hidden;
    border-left: 1px solid #ddd; font-size: 12.5px;
    display: flex; flex-direction: column;
  }
  #side header { padding: 12px 14px; background: #fff; border-bottom: 1px solid #eee; }
  #side header h3 { margin: 0 0 4px 0; }
  #cards { padding: 10px 12px; overflow: auto; flex: 1; }
  .card {
    background: #fff; border: 1px solid #e3e3e3; border-radius: 6px;
    padding: 10px 12px; margin-bottom: 10px; cursor: pointer;
    transition: box-shadow .15s, border-color .15s;
  }
  .card:hover { border-color: #bcd2ff; box-shadow: 0 2px 6px rgba(0,0,0,0.08); }
  .card.active { border-color: #e63946; box-shadow: 0 0 0 2px rgba(230,57,70,0.18); }
  .card .hd { font-size: 14px; }
  .card .hd .score { color: #888; float: right; font-size: 12px; }
  .card .row { color: #333; margin: 4px 0 6px 0; }
  .bar-wrap { margin: 6px 0; }
  .bar { display: flex; height: 10px; border-radius: 3px; overflow: hidden;
         background: #eee; }
  .bar .seg { display: block; height: 100%; }
  .elev-row { display: flex; gap: 12px; color: #555; font-size: 11.5px; margin-top: 6px; }
  .spark { display: block; width: 100%; height: 50px; }
  .actions { display: flex; gap: 6px; margin-top: 8px; }
  .actions button {
    flex: 1; padding: 4px 8px; border-radius: 4px; cursor: pointer;
    border: 1px solid #ccc; background: #f6f6f6; font-size: 12px;
  }
  .actions button:hover { background: #ececec; }
  .actions button.dl { background: #e63946; color: white; border-color: #e63946; }
  .actions button.dl:hover { background: #c81d2c; }
  .legend div { display:flex; align-items:center; margin: 2px 0; }
  .legend span.swatch { display:inline-block; width:14px; height:14px; margin-right:6px; }
  .swlegend { display: flex; flex-wrap: wrap; gap: 8px 12px; font-size: 11px;
              color: #555; margin-top: 4px; }
  .swlegend span.sw { display:inline-block; width:10px; height:10px;
                      margin-right:4px; vertical-align:middle; border-radius:2px; }
  .controls { padding: 8px 12px; background: #fff; border-bottom: 1px solid #eee; }
  .controls label { display: inline-block; margin-right: 10px; cursor: pointer; font-size: 12px; }
</style>
</head>
<body>
<div id="map"></div>
<div id="side">
  <header>
    <h3>Route suggestions</h3>
    <div class="legend">
      <div><span class="swatch" style="background:#e63946"></span>selected route</div>
      <div><span class="swatch" style="background:#f4a261"></span>other candidates</div>
      <div><span class="swatch" style="background:#000"></span>great-circle reference</div>
    </div>
    /*SWLEGEND*/
  </header>
  <div class="controls">
    <b>Basemap:</b>
    <label><input type="radio" name="bm" value="voyager" checked> street</label>
    <label><input type="radio" name="bm" value="topo"> terrain</label>
  </div>
  <div id="cards"></div>
</div>
<script src="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.js"></script>
<script>
const data = /*GEOJSON*/;
const cardsHTML = /*CARDS*/;
document.getElementById('cards').innerHTML = cardsHTML;

const bounds = new maplibregl.LngLatBounds();
function extend(coords) {
  if (typeof coords[0] === 'number') bounds.extend(coords);
  else coords.forEach(extend);
}
data.features.forEach(f => extend(f.geometry.coordinates));

const map = new maplibregl.Map({
  container: 'map',
  style: {
    version: 8,
    sources: {
      voyager: {
        type: 'raster',
        tiles: [
          'https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://b.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://c.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://d.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png'
        ],
        tileSize: 256,
        maxzoom: 19,
        attribution: '&copy; OpenStreetMap contributors &copy; CARTO'
      },
      topo: {
        type: 'raster',
        tiles: [
          'https://a.tile.opentopomap.org/{z}/{x}/{y}.png',
          'https://b.tile.opentopomap.org/{z}/{x}/{y}.png',
          'https://c.tile.opentopomap.org/{z}/{x}/{y}.png'
        ],
        tileSize: 256,
        maxzoom: 17,
        attribution: 'Map data &copy; OpenStreetMap contributors, SRTM | &copy; OpenTopoMap (CC-BY-SA)'
      }
    },
    layers: [
      { id: 'bm-voyager', type: 'raster', source: 'voyager', layout: { visibility: 'visible' } },
      { id: 'bm-topo',    type: 'raster', source: 'topo',    layout: { visibility: 'none' } }
    ]
  },
  bounds, fitBoundsOptions: { padding: 50 }
});

document.querySelectorAll('input[name="bm"]').forEach(r => {
  r.addEventListener('change', () => {
    const v = r.value;
    map.setLayoutProperty('bm-voyager', 'visibility', v === 'voyager' ? 'visible' : 'none');
    map.setLayoutProperty('bm-topo',    'visibility', v === 'topo'    ? 'visible' : 'none');
  });
});

let activeRank = 1;
function buildGpx(name, coords) {
  const pts = coords.map(c =>
    `    <trkpt lat="${(+c[1]).toFixed(7)}" lon="${(+c[0]).toFixed(7)}"/>`
  ).join('\n');
  return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="crowfly" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata><name>${name}</name></metadata>
  <trk><name>${name}</name><trkseg>
${pts}
  </trkseg></trk>
</gpx>
`;
}
function downloadRank(rank) {
  const route = data.features.find(f =>
    f.properties.kind === 'explore-route' && f.properties.rank === rank);
  if (!route) return;
  const gpx = buildGpx('rank ' + rank, route.geometry.coordinates);
  const blob = new Blob([gpx], { type: 'application/gpx+xml' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = `rank${rank}.gpx`; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 500);
}
function selectRank(rank) {
  activeRank = rank;
  document.querySelectorAll('.card').forEach(c =>
    c.classList.toggle('active', +c.dataset.rank === rank));
  // Update map paint to highlight chosen rank.
  if (map.getLayer('routes')) {
    map.setPaintProperty('routes', 'line-color',
      ['case', ['==', ['get', 'rank'], rank], '#e63946', '#f4a261']);
    map.setPaintProperty('routes', 'line-width',
      ['case', ['==', ['get', 'rank'], rank], 5, 2.5]);
  }
  if (map.getLayer('ends')) {
    map.setPaintProperty('ends', 'circle-color',
      ['case', ['==', ['get', 'rank'], rank], '#e63946', '#f4a261']);
    map.setPaintProperty('ends', 'circle-radius',
      ['case', ['==', ['get', 'rank'], rank], 7, 4]);
  }
  const route = data.features.find(f =>
    f.properties.kind === 'explore-route' && f.properties.rank === rank);
  if (route) {
    const b = new maplibregl.LngLatBounds();
    route.geometry.coordinates.forEach(c => b.extend(c));
    map.fitBounds(b, { padding: 60, duration: 700 });
  }
}

map.on('load', () => {
  map.addSource('data', { type: 'geojson', data });
  map.addLayer({
    id: 'nogo', type: 'fill', source: 'data',
    filter: ['==', ['get', 'kind'], 'no-go'],
    paint: { 'fill-color': '#ff0000', 'fill-opacity': 0.35 }
  });
  map.addLayer({
    id: 'reference', type: 'line', source: 'data',
    filter: ['==', ['get', 'kind'], 'explore-reference'],
    paint: {
      'line-color': '#222', 'line-width': 1.2, 'line-dasharray': [3, 3],
      'line-opacity': ['interpolate', ['linear'], ['get', 'rank'], 1, 0.7, 20, 0.15]
    }
  });
  map.addLayer({
    id: 'routes', type: 'line', source: 'data',
    filter: ['==', ['get', 'kind'], 'explore-route'],
    paint: {
      'line-color': ['case', ['==', ['get', 'rank'], 1], '#e63946', '#f4a261'],
      'line-width': ['case', ['==', ['get', 'rank'], 1], 5, 2.5],
      'line-opacity': ['interpolate', ['linear'], ['get', 'rank'], 1, 1.0, 20, 0.3]
    }
  });
  map.addLayer({
    id: 'ends', type: 'circle', source: 'data',
    filter: ['==', ['get', 'kind'], 'explore-end'],
    paint: {
      'circle-color': ['case', ['==', ['get', 'rank'], 1], '#e63946', '#f4a261'],
      'circle-radius': ['case', ['==', ['get', 'rank'], 1], 7, 4],
      'circle-stroke-color': '#fff', 'circle-stroke-width': 1.5
    }
  });
  map.addLayer({
    id: 'start', type: 'circle', source: 'data',
    filter: ['==', ['get', 'kind'], 'endpoint'],
    paint: {
      'circle-color': '#111', 'circle-radius': 8, 'circle-stroke-color': '#fff', 'circle-stroke-width': 2
    }
  });

  // Click on a route to highlight it; row click in table flies-to.
  map.on('click', 'routes', (e) => {
    const f = e.features[0];
    const p = f.properties;
    new maplibregl.Popup().setLngLat(e.lngLat)
      .setHTML(`<b>rank ${p.rank}</b><br/>bearing ${Math.round(p.bearing)}°<br/>` +
               `ref ${(+p.ref_km).toFixed(1)} km → real ${(+p.real_km).toFixed(1)} km<br/>` +
               `max dev ${Math.round(p.max_dev_m)} m, ferry ${(+p.ferry_km).toFixed(1)} km<br/>` +
               `score ${(+p.score).toFixed(1)}`)
      .addTo(map);
  });

  document.querySelectorAll('.card').forEach(card => {
    const rank = +card.dataset.rank;
    card.addEventListener('click', (e) => {
      // Don't trigger select on inner button clicks.
      if (e.target.tagName === 'BUTTON') return;
      selectRank(rank);
    });
    card.querySelector('.view').addEventListener('click', (e) => {
      e.stopPropagation();
      selectRank(rank);
    });
    card.querySelector('.dl').addEventListener('click', (e) => {
      e.stopPropagation();
      downloadRank(rank);
    });
  });
  // Highlight rank 1 by default.
  selectRank(1);
});
</script>
</body>
</html>
"#;

fn linestring_feature_props(kind: &str, name: &str, pts: &[LatLon], extra: Value) -> Value {
    let coords: Vec<Vec<f64>> = pts.iter().map(|p| vec![p.lon, p.lat]).collect();
    let mut props = serde_json::Map::new();
    props.insert("kind".to_string(), Value::String(kind.to_string()));
    props.insert("name".to_string(), Value::String(name.to_string()));
    if let Value::Object(m) = extra {
        for (k, v) in m {
            props.insert(k, v);
        }
    }
    json!({
        "type": "Feature",
        "properties": Value::Object(props),
        "geometry": {"type": "LineString", "coordinates": coords},
    })
}

fn point_feature_props(kind: &str, name: &str, p: LatLon, extra: Value) -> Value {
    let mut props = serde_json::Map::new();
    props.insert("kind".to_string(), Value::String(kind.to_string()));
    props.insert("name".to_string(), Value::String(name.to_string()));
    if let Value::Object(m) = extra {
        for (k, v) in m {
            props.insert(k, v);
        }
    }
    json!({
        "type": "Feature",
        "properties": Value::Object(props),
        "geometry": {"type": "Point", "coordinates": [p.lon, p.lat]},
    })
}

const HTML_TEMPLATE: &str = r#"<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>Straight-line route</title>
<link href="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.css" rel="stylesheet" />
<style>
  body { margin: 0; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
  #map { position: absolute; top: 0; bottom: 0; width: 100%; }
  .panel {
    position: absolute; background: rgba(255,255,255,0.95);
    padding: 10px 14px; z-index: 1; border-radius: 6px;
    box-shadow: 0 2px 8px rgba(0,0,0,0.2); font-size: 13px;
  }
  .info { top: 10px; left: 10px; max-width: 60vw; }
  .controls { top: 10px; right: 10px; }
  .controls label { display:block; margin: 2px 0; cursor: pointer; }
  .controls button {
    margin-top: 6px; padding: 6px 10px; cursor: pointer;
    background: #e63946; color: white; border: none; border-radius: 4px; font-weight: 600;
  }
  .controls button:hover { background: #c81d2c; }
  .legend { bottom: 30px; left: 10px; padding: 8px 12px; font-size: 12px; }
  .legend div { display: flex; align-items: center; margin: 2px 0; }
  .legend span.swatch { display: inline-block; width: 14px; height: 14px; margin-right: 6px; }
  .info .stats { margin-bottom: 6px; }
  .bar-wrap { margin: 6px 0 4px 0; }
  .bar { display: flex; height: 10px; border-radius: 3px; overflow: hidden; background: #eee; min-width: 280px; }
  .bar .seg { display: block; height: 100%; }
  .swlegend { display: flex; flex-wrap: wrap; gap: 6px 10px; font-size: 11px; color: #555; margin: 4px 0 6px 0; }
  .swlegend span.sw { display: inline-block; width: 10px; height: 10px; margin-right: 4px; vertical-align: middle; border-radius: 2px; }
  .elev-row { display: flex; gap: 12px; color: #555; font-size: 11.5px; margin: 6px 0 2px 0; }
  .spark { display: block; width: 100%; height: 60px; max-width: 480px; }
  .viol { margin-top: 4px; font-size: 12px; }
</style>
</head>
<body>
<div class="panel info" id="info">loading…</div>
<div class="panel controls">
  <div><b>Basemap</b></div>
  <label><input type="radio" name="bm" value="voyager" checked> Voyager (street)</label>
  <label><input type="radio" name="bm" value="topo"> OpenTopoMap (terrain)</label>
  <button id="dl">⬇ Download GPX</button>
  <div style="margin-top:6px;"><b>Surface</b></div>
  /*SWLEGEND*/
</div>
<div class="panel legend">
  <div><span class="swatch" style="background:#e63946"></span>routed path</div>
  <div><span class="swatch" style="background:#000"></span>reference line (great circle)</div>
  <div><span class="swatch" style="background:#3388ff;opacity:0.3"></span>corridor</div>
  <div><span class="swatch" style="background:#ff0000;opacity:0.4"></span>no-go zone</div>
  <div><span class="swatch" style="background:#ff8800"></span>long / ferry edge</div>
</div>
<div id="map"></div>
<script src="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.js"></script>
<script>
const data = /*GEOJSON*/;
const stats = /*STATS*/;
const bounds = new maplibregl.LngLatBounds();
function extend(coords) {
  if (typeof coords[0] === 'number') bounds.extend(coords);
  else coords.forEach(extend);
}
data.features.forEach(f => extend(f.geometry.coordinates));

const map = new maplibregl.Map({
  container: 'map',
  style: {
    version: 8,
    sources: {
      voyager: {
        type: 'raster',
        tiles: [
          'https://a.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://b.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://c.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png',
          'https://d.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}.png'
        ],
        tileSize: 256,
        maxzoom: 19,
        attribution: '&copy; OpenStreetMap contributors &copy; CARTO'
      },
      topo: {
        type: 'raster',
        tiles: [
          'https://a.tile.opentopomap.org/{z}/{x}/{y}.png',
          'https://b.tile.opentopomap.org/{z}/{x}/{y}.png',
          'https://c.tile.opentopomap.org/{z}/{x}/{y}.png'
        ],
        tileSize: 256,
        maxzoom: 17,
        attribution: 'Map data &copy; OpenStreetMap contributors, SRTM | &copy; OpenTopoMap (CC-BY-SA)'
      }
    },
    layers: [
      { id: 'bm-voyager', type: 'raster', source: 'voyager', layout: { visibility: 'visible' } },
      { id: 'bm-topo',    type: 'raster', source: 'topo',    layout: { visibility: 'none' } }
    ]
  },
  bounds, fitBoundsOptions: { padding: 50 }
});

document.querySelectorAll('input[name="bm"]').forEach(r => {
  r.addEventListener('change', () => {
    const v = r.value;
    map.setLayoutProperty('bm-voyager', 'visibility', v === 'voyager' ? 'visible' : 'none');
    map.setLayoutProperty('bm-topo',    'visibility', v === 'topo'    ? 'visible' : 'none');
  });
});

function buildGpx(name, coords) {
  const pts = coords.map(c =>
    `    <trkpt lat="${(+c[1]).toFixed(7)}" lon="${(+c[0]).toFixed(7)}"/>`
  ).join('\n');
  return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="crowfly" xmlns="http://www.topografix.com/GPX/1/1">
  <metadata><name>${name}</name></metadata>
  <trk><name>${name}</name><trkseg>
${pts}
  </trkseg></trk>
</gpx>
`;
}

document.getElementById('dl').addEventListener('click', () => {
  const route = data.features.find(f => f.properties.kind === 'route');
  if (!route) { alert('no route to export'); return; }
  const gpx = buildGpx('crowfly route', route.geometry.coordinates);
  const blob = new Blob([gpx], { type: 'application/gpx+xml' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = 'route.gpx'; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 500);
});

map.on('load', () => {
  map.addSource('data', { type: 'geojson', data });
  map.addLayer({
    id: 'corridor', type: 'fill', source: 'data',
    filter: ['==', ['get', 'kind'], 'corridor'],
    paint: { 'fill-color': '#3388ff', 'fill-opacity': 0.12, 'fill-outline-color': '#3388ff' }
  });
  map.addLayer({
    id: 'nogo', type: 'fill', source: 'data',
    filter: ['==', ['get', 'kind'], 'no-go'],
    paint: { 'fill-color': '#ff0000', 'fill-opacity': 0.35, 'fill-outline-color': '#aa0000' }
  });
  map.addLayer({
    id: 'reference', type: 'line', source: 'data',
    filter: ['==', ['get', 'kind'], 'reference'],
    paint: { 'line-color': '#000', 'line-width': 2, 'line-dasharray': [3, 3] }
  });
  map.addLayer({
    id: 'longedge', type: 'line', source: 'data',
    filter: ['==', ['get', 'kind'], 'long-edge'],
    paint: { 'line-color': '#ff8800', 'line-width': 4, 'line-dasharray': [1, 1] }
  });
  map.addLayer({
    id: 'route', type: 'line', source: 'data',
    filter: ['==', ['get', 'kind'], 'route'],
    paint: { 'line-color': '#e63946', 'line-width': 4 }
  });
  map.addLayer({
    id: 'endpoints', type: 'circle', source: 'data',
    filter: ['==', ['get', 'kind'], 'endpoint'],
    paint: { 'circle-color': '#111', 'circle-radius': 7, 'circle-stroke-color': '#fff', 'circle-stroke-width': 2 }
  });
  document.getElementById('info').innerHTML = stats;
});
</script>
</body>
</html>
"#;
