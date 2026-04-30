//! A* routing on the corridor-bounded graph.
//!
//! Cost combines raw edge length with a quadratic penalty for cross-track
//! deviation from the reference line `a -> b`. Computing deviation in the cost
//! closure (rather than baking it into edges) lets one big graph serve many
//! candidate lines, which is what `explore` mode needs.

use crate::geodesy::{cross_track, haversine, LatLon};
use crate::osm::{EdgeData, Graph};
use anyhow::{anyhow, Result};
use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;

#[derive(Clone, Debug)]
pub struct RouteParams {
    pub modes: u8,
    pub half_width_m: f64,
    pub alpha: f64,
}

#[derive(Clone, Debug)]
pub struct RouteResult {
    pub points: Vec<LatLon>,
    pub edges: Vec<EdgeData>,
    pub total_length_m: f64,
}

/// Average lat/lon of an edge's endpoints. Adequate for short OSM ways
/// where curvature is negligible.
fn midpoint(p0: LatLon, p1: LatLon) -> LatLon {
    LatLon::new((p0.lat + p1.lat) * 0.5, (p0.lon + p1.lon) * 0.5)
}

pub fn shortest(
    graph: &Graph,
    start: NodeIndex,
    end: NodeIndex,
    line_a: LatLon,
    line_b: LatLon,
    params: &RouteParams,
) -> Result<RouteResult> {
    let target_pos = graph.graph[end];
    let modes = params.modes;
    let alpha = params.alpha;
    let half = params.half_width_m.max(1.0);
    let g = &graph.graph;

    let cost = |p0: LatLon, p1: LatLon, e: &EdgeData| -> f64 {
        if (e.modes & modes) == 0 {
            return f64::INFINITY;
        }
        let mid = midpoint(p0, p1);
        let dev = cross_track(mid, line_a, line_b).abs();
        let rel = dev / half;
        e.length_m * (1.0 + alpha * rel * rel)
    };

    let result = astar(
        g,
        start,
        |n| n == end,
        |er| {
            let p0 = g[er.source()];
            let p1 = g[er.target()];
            cost(p0, p1, er.weight())
        },
        |n| haversine(g[n], target_pos),
    );

    let (_, path) = result.ok_or_else(|| {
        anyhow!("no route found between selected nodes — try a wider corridor or different modes")
    })?;

    let mut points = Vec::with_capacity(path.len());
    for &ni in &path {
        points.push(g[ni]);
    }
    let mut edges = Vec::with_capacity(path.len().saturating_sub(1));
    let mut total_length = 0.0;
    for w in path.windows(2) {
        let p0 = g[w[0]];
        let p1 = g[w[1]];
        let mut best: Option<(EdgeData, f64)> = None;
        for er in g.edges_connecting(w[0], w[1]) {
            let e = er.weight();
            if (e.modes & modes) == 0 {
                continue;
            }
            let c = cost(p0, p1, e);
            if best.map_or(true, |(_, bc)| c < bc) {
                best = Some((*e, c));
            }
        }
        if let Some((e, _)) = best {
            total_length += e.length_m;
            edges.push(e);
        }
    }

    Ok(RouteResult {
        points,
        edges,
        total_length_m: total_length,
    })
}
