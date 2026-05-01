//! A* routing on the corridor-bounded graph.
//!
//! Cost combines raw edge length with a quadratic penalty for cross-track
//! deviation from the reference line `a -> b`. Computing deviation in the cost
//! closure (rather than baking it into edges) lets one big graph serve many
//! candidate lines, which is what `explore` mode needs.
//!
//! Edges whose mid-point deviates beyond `corridor_max_m` are skipped (cost =
//! +∞). This both enforces the corridor constraint at the routing layer and
//! prunes the A* frontier to the corridor envelope, which is the dominant
//! perf win for tight corridors over a country-sized graph.
//!
//! We use a custom A* with FxHash-backed score maps + lazy deletion, instead
//! of petgraph::algo::astar. petgraph's astar uses std::HashMap (SipHash) and
//! a separate visit map; on a 16M-node graph the hashing dominates the
//! routing inner loop.

use crate::geodesy::{CrossTrack, HaversineToTarget, LatLon};
use crate::osm::{EdgeData, Graph, SURFACE_PAVED};
use anyhow::{anyhow, Result};
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::visit::EdgeRef;
use rustc_hash::FxHashMap;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Debug)]
pub struct RouteParams {
    pub modes: u8,
    pub half_width_m: f64,
    pub alpha: f64,
    /// Hard cap on per-edge cross-track deviation. Edges whose midpoint sits
    /// further than this from the reference line are skipped. Set this to
    /// `half_width_m` for strict corridor compliance.
    pub corridor_max_m: f64,
    /// If true, restrict routing to edges classified as paved (asphalt /
    /// concrete / paving stones / cobblestone / paved tracks). Other surfaces
    /// — gravel, path, unpaved, ferry — are skipped.
    pub paved_only: bool,
}

#[derive(Clone, Debug)]
pub struct RouteResult {
    pub points: Vec<LatLon>,
    pub edges: Vec<EdgeData>,
    pub total_length_m: f64,
}

fn midpoint(p0: LatLon, p1: LatLon) -> LatLon {
    LatLon::new((p0.lat + p1.lat) * 0.5, (p0.lon + p1.lon) * 0.5)
}

#[derive(Copy, Clone, Debug)]
struct Frontier {
    f: f64,
    node: NodeIndex,
}

impl PartialEq for Frontier {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.node == other.node
    }
}

impl Eq for Frontier {}

impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on f-score: invert the usual comparison.
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.node.index().cmp(&other.node.index()))
    }
}

impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Custom A* over an undirected graph with f64 edge costs and a lower-bound
/// heuristic. Returns `(total_cost, path)` on success. Edge costs that return
/// non-finite (∞ / NaN) are treated as forbidden and skipped.
fn astar_fx<F, H>(
    graph: &UnGraph<LatLon, EdgeData>,
    start: NodeIndex,
    end: NodeIndex,
    mut edge_cost: F,
    mut heuristic: H,
) -> Option<(f64, Vec<NodeIndex>)>
where
    F: FnMut(petgraph::graph::EdgeReference<'_, EdgeData>) -> f64,
    H: FnMut(NodeIndex) -> f64,
{
    let mut g_score: FxHashMap<NodeIndex, f64> = FxHashMap::default();
    let mut came_from: FxHashMap<NodeIndex, NodeIndex> = FxHashMap::default();
    let mut open: BinaryHeap<Frontier> = BinaryHeap::new();

    g_score.insert(start, 0.0);
    open.push(Frontier {
        f: heuristic(start),
        node: start,
    });

    while let Some(Frontier { f, node }) = open.pop() {
        if node == end {
            let total = g_score[&node];
            let mut path = vec![node];
            let mut cur = node;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some((total, path));
        }
        let cur_g = match g_score.get(&node) {
            Some(&g) => g,
            None => continue,
        };
        // Lazy deletion: if a better f for this node has already been popped,
        // the heap entry is stale.
        if f > cur_g + heuristic(node) + 1e-9 {
            continue;
        }
        for er in graph.edges(node) {
            let cost = edge_cost(er);
            if !cost.is_finite() {
                continue;
            }
            let next = er.target();
            let tentative = cur_g + cost;
            let prev_g = g_score.get(&next).copied().unwrap_or(f64::INFINITY);
            if tentative < prev_g {
                g_score.insert(next, tentative);
                came_from.insert(next, node);
                open.push(Frontier {
                    f: tentative + heuristic(next),
                    node: next,
                });
            }
        }
    }
    None
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
    let inv_half = 1.0 / half;
    let corridor_max = params.corridor_max_m.max(half);
    let g = &graph.graph;
    let xt = CrossTrack::new(line_a, line_b);
    let heur = HaversineToTarget::new(target_pos);

    let paved_only = params.paved_only;
    let cost = |p0: LatLon, p1: LatLon, e: &EdgeData| -> f64 {
        if (e.modes & modes) == 0 {
            return f64::INFINITY;
        }
        if paved_only && e.surface != SURFACE_PAVED {
            return f64::INFINITY;
        }
        let mid = midpoint(p0, p1);
        let dev = xt.abs(mid);
        if dev > corridor_max {
            return f64::INFINITY;
        }
        let rel = dev * inv_half;
        e.length_m * (1.0 + alpha * rel * rel)
    };

    let result = astar_fx(
        g,
        start,
        end,
        |er| {
            let p0 = g[er.source()];
            let p1 = g[er.target()];
            cost(p0, p1, er.weight())
        },
        |n| heur.distance(g[n]),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::haversine;
    use crate::osm::{NodePoint, MODE_BIKE, MODE_FOOT, SURFACE_PAVED};
    use petgraph::graph::UnGraph;
    use rustc_hash::FxHashMap;

    /// Build a synthetic graph that routes A→B via either an inside-corridor
    /// path or a far detour. Used to verify the corridor cutoff hard-skips
    /// out-of-corridor edges and that successful routes never exceed the
    /// half-width.
    fn synthetic_graph(detour_offset_deg: f64) -> Graph {
        let mut g: UnGraph<LatLon, EdgeData> = UnGraph::new_undirected();
        // A and B are 1° apart along the equator (~111 km). Two intermediate
        // nodes form a north-detour offset by `detour_offset_deg`.
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 1.0));
        let m1 = g.add_node(LatLon::new(detour_offset_deg, 0.33));
        let m2 = g.add_node(LatLon::new(detour_offset_deg, 0.66));
        let edge = |g: &mut UnGraph<LatLon, EdgeData>, x: NodeIndex, y: NodeIndex| {
            let length = haversine(g[x], g[y]);
            g.add_edge(
                x,
                y,
                EdgeData {
                    length_m: length,
                    modes: MODE_FOOT | MODE_BIKE,
                    surface: SURFACE_PAVED,
                    way_id: 1,
                },
            );
        };
        edge(&mut g, a, m1);
        edge(&mut g, m1, m2);
        edge(&mut g, m2, b);
        let pts: Vec<NodePoint> = g
            .node_indices()
            .map(|ni| NodePoint {
                idx: ni,
                lat: g[ni].lat,
                lon: g[ni].lon,
            })
            .collect();
        let rtree = rstar::RTree::bulk_load(pts);
        let n = g.node_count();
        let components: Vec<u32> = (0..n as u32).map(|_| 0).collect();
        Graph {
            graph: g,
            osm_to_idx: FxHashMap::default(),
            components,
            rtree,
        }
    }

    #[test]
    fn corridor_cutoff_rejects_out_of_corridor_paths() {
        // Detour ~111 km north of the line A→B. Half-width 5 km: only path
        // available is well outside corridor → routing must fail.
        let graph = synthetic_graph(1.0);
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(0.0, 1.0);
        let start = NodeIndex::new(0);
        let end = NodeIndex::new(1);
        let params = RouteParams {
            modes: MODE_FOOT,
            half_width_m: 5_000.0,
            alpha: 4.0,
            corridor_max_m: 5_000.0,
            paved_only: false,
        };
        let result = shortest(&graph, start, end, a, b, &params);
        assert!(
            result.is_err(),
            "expected no route — detour is far outside corridor"
        );
    }

    #[test]
    fn successful_routes_respect_corridor() {
        // Detour offset within corridor: route should succeed and every
        // mid-point of the resulting path must be within the half-width.
        let graph = synthetic_graph(0.02); // ~2.2 km north
        let a = LatLon::new(0.0, 0.0);
        let b = LatLon::new(0.0, 1.0);
        let start = NodeIndex::new(0);
        let end = NodeIndex::new(1);
        let params = RouteParams {
            modes: MODE_FOOT,
            half_width_m: 5_000.0,
            alpha: 4.0,
            corridor_max_m: 5_000.0,
            paved_only: false,
        };
        let r = shortest(&graph, start, end, a, b, &params).expect("route");
        let xt = CrossTrack::new(a, b);
        for w in r.points.windows(2) {
            let mid = midpoint(w[0], w[1]);
            let dev = xt.abs(mid);
            assert!(
                dev <= params.half_width_m,
                "edge midpoint deviation {dev:.0}m exceeds half-width"
            );
        }
        // And every visited vertex.
        for p in &r.points {
            let dev = xt.abs(*p);
            assert!(
                dev <= params.half_width_m * 1.05,
                "vertex deviation {dev:.0}m exceeds half-width (with 5% slack for snap)"
            );
        }
    }
}
