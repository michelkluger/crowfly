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
//! We use a custom A* with FxHash-backed state + lazy deletion, instead of
//! petgraph::algo::astar: on a 16M-node graph the hashing and heap traffic
//! dominate the routing inner loop.
//!
//! Inner-loop economics (in rough order of importance):
//! - The heuristic is scaled to the *tightest admissible* bound, computed
//!   from the actual multiplier tables. A slack heuristic inflates the
//!   explored ellipse quadratically, so this is the dominant lever on search
//!   size for bike/mtb routing.
//! - A* only stops at nodes where the route can branch (`Graph::is_junction`,
//!   plus the start and end). ~84 % of OSM nodes are mid-road shape points
//!   with exactly two neighbours; since U-turns are forbidden, a route that
//!   enters such a chain must follow it, so the search walks each chain in
//!   one go — charging every edge as before — instead of pushing every shape
//!   point through the heap and the hash map (~6× fewer of each).
//! - Per-stop-node state (g, cached h, walk origin) lives in one FxHashMap
//!   so each relaxation costs a single hash probe, and heap entries carry
//!   their g so stale entries are detected without recomputing h on pop.
//! - An edge's midpoint deviation is the mean of its endpoint deviations
//!   (the deviation field is smooth — centimetre-level error at OSM edge
//!   lengths), so a walk evaluates one spherical cross-track per node.
//! - Turn-penalty bearings use the tangent-plane approximation: one cos +
//!   one atan2 per edge instead of two full great-circle bearings.

use crate::geodesy::{fast_bearing_rad, CrossTrack, HaversineToTarget, LatLon};
use crate::osm::{
    EdgeData, Graph, BATTR_CYCLE_NETWORK, BATTR_DESIGNATED, BCLASS_CYCLEWAY, BCLASS_FERRY,
    BCLASS_MASK, BCLASS_OTHER, BCLASS_PATH, BCLASS_PRIMARY, BCLASS_RESIDENTIAL, BCLASS_SECONDARY,
    BCLASS_SERVICE, BCLASS_TERTIARY, BCLASS_TRACK, MODE_BIKE, MODE_MTB, SURFACE_FERRY,
    SURFACE_GRAVEL, SURFACE_OTHER, SURFACE_PATH, SURFACE_PAVED, SURFACE_UNPAVED,
};
use anyhow::{anyhow, Result};
use petgraph::graph::{EdgeIndex, EdgeReference, NodeIndex};
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
    /// Graph nodes visited in order, parallel to `points`. Allows
    /// post-routing graph-aware operations (loop pruning, leg continuation)
    /// without re-snapping LatLon points to nodes.
    pub node_indices: Vec<NodeIndex>,
}

/// Test-only helper: the routing cost path now derives an edge's corridor
/// deviation from its cached endpoint deviations instead of materialising
/// the midpoint.
#[cfg(test)]
fn midpoint(p0: LatLon, p1: LatLon) -> LatLon {
    LatLon::new((p0.lat + p1.lat) * 0.5, (p0.lon + p1.lon) * 0.5)
}

#[derive(Copy, Clone, Debug)]
struct Frontier {
    f: f64,
    /// g-score at push time. Lets the pop loop detect stale (lazily-deleted)
    /// entries by comparing against the node's current g — no heuristic
    /// recomputation — and serves as the tie-breaker.
    g: f64,
    node: NodeIndex,
}

impl PartialEq for Frontier {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.g == other.g && self.node == other.node
    }
}

impl Eq for Frontier {}

impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on f-score: invert the usual comparison. Among equal f,
        // prefer the entry with the *larger* g (deeper along its path, i.e.
        // closer to the goal) — the standard A* tie-break that avoids
        // ping-ponging across equal-f frontiers in grid-like road networks.
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.g.partial_cmp(&other.g).unwrap_or(Ordering::Equal))
            .then_with(|| self.node.index().cmp(&other.node.index()))
    }
}

impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Search state of a node A* stops at: best-known g, cached heuristic, and
/// the walk that achieved it. One map entry per stop node keeps each
/// relaxation to a single hash probe, and the cached h means re-improving a
/// node never recomputes the heuristic.
struct NodeState {
    g: f64,
    h: f64,
    /// Origin of the walk that last improved this node (`start` for itself).
    from: NodeIndex,
    /// That walk's first edge; None for `start`.
    via: Option<EdgeIndex>,
    /// Node just before this one on the best path — the turn-penalty and
    /// U-turn reference for leaving it. `start_prev` for `start`.
    prev: Option<NodeIndex>,
}

/// Splice out near-revisits: subsequences where the polyline returns within
/// `max_radius_m` of an earlier point after walking at least `min_loop_m`.
/// Unlike `prune_revisits`, this is a *geometric* pass — the two endpoints
/// of the spliced loop are different graph nodes, so the resulting polyline
/// has a tiny visual jump (≤ `max_radius_m`) where the loop used to be.
/// In exchange, we kill the "out-and-back via a parallel cycleway / service
/// road" pattern that single-leg A* cannot avoid (the lobe IS optimal under
/// edge-local cost, but the user perceives it as garbage).
///
/// The `nodes`/`edges` invariant (`edges.len() + 1 == nodes.len()`) is
/// preserved by skipping the bridging edge entirely; total length is
/// recomputed downstream from the surviving edges, so it under-reports the
/// route by at most `max_radius_m` per pruned tail — acceptable cleanup
/// noise.
pub fn prune_geometric_loops(
    nodes: &mut Vec<NodeIndex>,
    points: &mut Vec<LatLon>,
    edges: &mut Vec<EdgeData>,
    max_radius_m: f64,
    min_loop_m: f64,
) -> usize {
    let n = points.len();
    if n < 20 {
        return 0;
    }
    debug_assert_eq!(nodes.len(), n);
    debug_assert_eq!(edges.len() + 1, n);
    let mut cum = vec![0.0_f64; n];
    for i in 1..n {
        cum[i] = cum[i - 1] + crate::geodesy::haversine(points[i - 1], points[i]);
    }
    let mut keep = vec![true; n];
    let mut i = 0;
    while i < n {
        if !keep[i] {
            i += 1;
            continue;
        }
        // Scan forward; remember the latest match so the longest loop wins.
        let mut latest: Option<usize> = None;
        let max_scan = (cum[i] + min_loop_m * 20.0).min(cum[n - 1]);
        let mut j = i + 5;
        while j < n {
            if !keep[j] {
                j += 1;
                continue;
            }
            if cum[j] > max_scan {
                break;
            }
            if cum[j] - cum[i] < min_loop_m {
                j += 1;
                continue;
            }
            let d = crate::geodesy::haversine(points[i], points[j]);
            if d <= max_radius_m {
                latest = Some(j);
            }
            j += 1;
        }
        if let Some(j) = latest {
            for k in (i + 1)..=j {
                keep[k] = false;
            }
        }
        i += 1;
    }
    let removed = keep.iter().filter(|&&k| !k).count();
    if removed == 0 {
        return 0;
    }
    // Build the surviving sequence, synthesizing a bridge edge across each
    // splice so the (edges.len() + 1 == nodes.len()) invariant survives and
    // downstream length / surface accounting stays correct (under-counting
    // each loop by ≈ haversine(loop_entry, loop_exit) ≤ max_radius_m).
    let mut new_nodes = Vec::with_capacity(n - removed);
    let mut new_points = Vec::with_capacity(n - removed);
    let mut new_edges = Vec::with_capacity(n - removed);
    let mut last_kept: Option<usize> = None;
    for k in 0..n {
        if !keep[k] {
            continue;
        }
        if let Some(prev) = last_kept {
            if prev + 1 == k {
                // Adjacent kept nodes: re-use the original edge.
                new_edges.push(edges[prev]);
            } else {
                // Spliced gap: synthesize a short bridge edge. We carry
                // surface/modes/bike_attrs from one of the dropped edges so
                // the bridge inherits roughly the same character as the
                // road it was bypassing.
                let bridge_length = crate::geodesy::haversine(points[prev], points[k]);
                let template = if prev < edges.len() {
                    edges[prev]
                } else {
                    *edges.last().unwrap()
                };
                new_edges.push(EdgeData {
                    length_m: bridge_length,
                    modes: template.modes,
                    surface: template.surface,
                    bike_attrs: template.bike_attrs,
                });
            }
        }
        new_nodes.push(nodes[k]);
        new_points.push(points[k]);
        last_kept = Some(k);
    }
    *nodes = new_nodes;
    *points = new_points;
    *edges = new_edges;
    removed
}

/// Splice out cycles in a graph-path. If the same `NodeIndex` appears twice
/// at indices `i < j`, drop nodes `(i, j]` (and the corresponding edges) so
/// the surviving path goes ...→nodes[i]→nodes[j+1]→...  By construction
/// nodes[i] == nodes[j], so the edge that previously connected nodes[j] to
/// nodes[j+1] now connects nodes[i] to nodes[j+1] — a real graph edge, no
/// fabricated geometry.
///
/// Returns the number of nodes removed; callers use it as a "tails were
/// pruned" signal.
pub fn prune_revisits(
    nodes: &mut Vec<NodeIndex>,
    points: &mut Vec<LatLon>,
    edges: &mut Vec<EdgeData>,
) -> usize {
    if nodes.len() < 3 {
        return 0;
    }
    debug_assert_eq!(nodes.len(), points.len());
    debug_assert_eq!(edges.len() + 1, nodes.len());
    let mut keep_node = vec![true; nodes.len()];
    let mut keep_edge = vec![true; edges.len()];
    let mut first_seen: FxHashMap<NodeIndex, usize> = FxHashMap::default();
    let mut removed = 0_usize;
    for i in 0..nodes.len() {
        if !keep_node[i] {
            continue;
        }
        if let Some(&earlier) = first_seen.get(&nodes[i]) {
            // Cycle detected: nodes[earlier..=i] forms a loop returning to
            // the same graph node. Drop the interior (earlier+1 .. i) plus
            // the duplicate at i. The edges spanning the loop interior are
            // edges[earlier .. i] — all dropped. The edge previously sitting
            // at edges[i] (connecting nodes[i] → nodes[i+1]) still applies
            // to nodes[earlier] → nodes[i+1] because nodes[earlier] == nodes[i].
            for k in (earlier + 1)..=i {
                if keep_node[k] {
                    keep_node[k] = false;
                    removed += 1;
                }
            }
            for k in earlier..i {
                keep_edge[k] = false;
            }
            // Any node that was logged in `first_seen` between `earlier` and
            // `i` is no longer in the surviving path, so its entry should be
            // discarded.  Scanning the small region is cheap.
            for k in (earlier + 1)..i {
                if first_seen.get(&nodes[k]).copied() == Some(k) {
                    first_seen.remove(&nodes[k]);
                }
            }
        } else {
            first_seen.insert(nodes[i], i);
        }
    }
    if removed == 0 {
        return 0;
    }
    let mut new_nodes = Vec::with_capacity(nodes.len() - removed);
    let mut new_points = Vec::with_capacity(points.len() - removed);
    for i in 0..nodes.len() {
        if keep_node[i] {
            new_nodes.push(nodes[i]);
            new_points.push(points[i]);
        }
    }
    let mut new_edges = Vec::with_capacity(edges.len().saturating_sub(removed));
    for i in 0..edges.len() {
        if keep_edge[i] {
            new_edges.push(edges[i]);
        }
    }
    *nodes = new_nodes;
    *points = new_points;
    *edges = new_edges;
    removed
}

/// Single-leg A* with no caller-supplied predecessor. Convenience wrapper
/// around [`shortest_with_start_prev`] for between-A→B and one-shot queries.
pub fn shortest(
    graph: &Graph,
    start: NodeIndex,
    end: NodeIndex,
    line_a: LatLon,
    line_b: LatLon,
    params: &RouteParams,
) -> Result<RouteResult> {
    shortest_with_start_prev(graph, start, end, None, line_a, line_b, params)
}

/// Same as [`shortest`] but lets the caller supply a virtual predecessor of
/// `start`. Used by [`route_loop`] to thread the U-turn-forbid and turn
/// penalty across leg boundaries.
pub fn shortest_with_start_prev(
    graph: &Graph,
    start: NodeIndex,
    end: NodeIndex,
    start_prev: Option<NodeIndex>,
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
    let elevations = graph.elevations_m.as_deref();
    let bike_active = (modes & MODE_BIKE) != 0;
    let mtb_active = (modes & MODE_MTB) != 0;
    // When MTB is in the user's mode mask, the cyclist is signalling tolerance
    // for rough surfaces. Pure bike-mode is treated as road-bike leaning:
    // avoid footpaths / rough tracks unless the alternative is clearly worse.
    let rough_scale: f64 = if mtb_active && !bike_active {
        0.25
    } else if mtb_active {
        0.5
    } else {
        1.0
    };
    // Per-surface "fictional metres added per metre of descent" when bike or
    // mtb is in the active mode mask. Steeper drops on path/footway/steps
    // become disproportionately expensive, so the router prefers a longer
    // rideable alternative on the way down. Uphill traversal is unaffected
    // (descent ≤ 0); ascent gets its own grade-squared penalty below.
    let descent_factor = |surface: u8| -> f64 {
        if !bike_active && !mtb_active {
            return 0.0;
        }
        let base = match surface {
            SURFACE_PAVED | SURFACE_FERRY => 0.0,
            SURFACE_GRAVEL => 1.0,
            SURFACE_UNPAVED => 8.0,
            SURFACE_OTHER => 4.0,
            SURFACE_PATH => 25.0,
            _ => 0.0,
        };
        base * rough_scale
    };
    // Per-class cost multiplier applied when bike (or mtb) is in the user's
    // mode mask. The paved-road classes stay compressed so the router doesn't
    // attach to every short cycleway spur. Path/track are deliberately much
    // harsher for pure bike mode: on a road bike, MTB-ish paths are a last
    // resort, not a nice shortcut. MTB mode keeps those classes cheap.
    let bike_class_mult = |class: u8| -> f64 {
        match class {
            BCLASS_CYCLEWAY => 1.00,
            BCLASS_RESIDENTIAL => 1.00,
            BCLASS_TERTIARY => 1.00,
            BCLASS_SERVICE => 1.05,
            BCLASS_SECONDARY => 1.15,
            BCLASS_PRIMARY => 1.40,
            BCLASS_TRACK => {
                if mtb_active {
                    1.00
                } else {
                    1.35
                }
            }
            BCLASS_PATH => {
                if mtb_active && !bike_active {
                    1.00
                } else if mtb_active {
                    1.10
                } else {
                    2.60
                }
            }
            BCLASS_FERRY | BCLASS_OTHER => 1.0,
            _ => 1.0,
        }
    };
    // Surface multiplier that applies *always* when bike/mtb are in play, not
    // just on descent — riding 10 km of path is slow even on the flat. For
    // pure bike mode, gravel remains acceptable, but unknown/path/unpaved are
    // expensive enough that a sane paved-road detour should win.
    let surface_mult = |surface: u8| -> f64 {
        if !bike_active && !mtb_active {
            return 1.0;
        }
        let base = match surface {
            SURFACE_PAVED | SURFACE_FERRY => 1.0,
            SURFACE_GRAVEL => 1.12,
            SURFACE_UNPAVED => 1.85,
            SURFACE_OTHER => 1.35,
            SURFACE_PATH => 2.20,
            _ => 1.0,
        };
        // Soften with rough_scale: subtract the excess proportionally so MTB
        // mode doesn't pay almost any "always" surface tax, while bike mode
        // pays the full rate.
        1.0 + (base - 1.0) * rough_scale
    };
    // Designated and signed-network bonuses kept small so they nudge ranking
    // without making the router go out of its way to attach to a marked
    // spur. These are the only factors that can price an edge *below* its
    // physical length, so they also bound the admissible heuristic below.
    const DESIGNATED_BONUS: f64 = 0.98;
    const NETWORK_BONUS: f64 = 0.99;
    // Turn penalty (metres added per radian of bearing change at a junction)
    // applied when bike/mtb is active. Linear in |Δbearing|: a 90° turn costs
    // +TURN_PEN·π/2 m, a 180° pivot costs +TURN_PEN·π m. With TURN_PEN=18:
    //   30°  → +9 m   (gentle bend, almost free)
    //   60°  → +19 m  (real turn, tolerated by short routes)
    //   90°  → +28 m  (right-angle, router prefers a smoother alternative)
    //   180° → +57 m  (hairpin / U-turn, strongly discouraged)
    // This is what kills the side-spur attaching pathology — a 200 m cycleway
    // detour that needs two 90° turns to enter and exit now costs +56 m of
    // "fictional length" beyond the cycleway itself, dwarfing the small
    // class bonus.
    const TURN_PEN: f64 = 100.0;
    let turn_active = bike_active || mtb_active;
    // Cost of an edge given its precomputed |corridor deviation| and the
    // incoming bearing at p0 (None = no predecessor / zero-length entry
    // segment). Mode/surface rejection checks live here so both the search
    // closure and the parallel-edge reconstruction pass share them.
    let cost = |e: &EdgeData,
                dev: f64,
                b_in: Option<f64>,
                p0: LatLon,
                p1: LatLon,
                elev_pair: Option<(f32, f32)>|
     -> f64 {
        if (e.modes & modes) == 0 {
            return f64::INFINITY;
        }
        if paved_only && e.surface != SURFACE_PAVED {
            return f64::INFINITY;
        }
        if dev > corridor_max {
            return f64::INFINITY;
        }
        let rel = dev * inv_half;
        let mut weight = 1.0 + alpha * rel * rel;
        if bike_active || mtb_active {
            let class = e.bike_attrs & BCLASS_MASK;
            let mut bm = bike_class_mult(class);
            if (e.bike_attrs & BATTR_DESIGNATED) != 0 {
                bm *= DESIGNATED_BONUS;
            }
            if (e.bike_attrs & BATTR_CYCLE_NETWORK) != 0 {
                bm *= NETWORK_BONUS;
            }
            weight *= bm;
            weight *= surface_mult(e.surface);
            // If the user has bike enabled, a pure foot edge is a push/carry
            // segment, not normal riding. Keep it possible for mixed
            // bike+hike plans, but make a paved rideable detour strongly win
            // for road-bike-like searches. MTB users are more likely to have
            // intentionally enabled rough non-road traversal, so don't apply
            // the road-bike penalty there.
            if bike_active && !mtb_active && (e.modes & MODE_BIKE) == 0 {
                weight *= 4.0;
            }
        }
        let mut c = e.length_m * weight;
        if let Some(b_in) = b_in {
            if p0 != p1 {
                let b_out = fast_bearing_rad(p0, p1);
                let mut d = (b_out - b_in).abs();
                if d > std::f64::consts::PI {
                    d = 2.0 * std::f64::consts::PI - d;
                }
                c += TURN_PEN * d;
            }
        }
        if let Some((zs, zt)) = elev_pair {
            if zs.is_finite() && zt.is_finite() {
                let drop_m = (zs - zt) as f64;
                if drop_m > 1.0 {
                    c += drop_m * descent_factor(e.surface);
                }
                // Ascent grade penalty with a hinge at ~4 %: stays ~0 on
                // gentle rollers and ramps quadratically above 4 % so steep
                // walls become disproportionately expensive. With k=80 (bike)
                // / k=45 (mtb-only):
                //   6 %  ≈ +3 %      (rolling — almost free)
                //   8 %  ≈ +13 %     (noticeable)
                //   10 % ≈ +29 %     (avoidable if any flatter detour exists)
                //   15 % ≈ +97 %     (router strongly prefers a longer route)
                //   20 % ≈ +205 %    (effectively forbidden unless no choice)
                // Disabled for foot-only routing (hikers don't gain from it).
                if (bike_active || mtb_active) && drop_m < -1.0 && e.length_m > 1.0 {
                    let rise_m = -drop_m;
                    let grade = rise_m / e.length_m;
                    const HINGE: f64 = 0.04;
                    let excess = (grade - HINGE).max(0.0);
                    let k = if mtb_active && !bike_active {
                        45.0
                    } else {
                        80.0
                    };
                    c += e.length_m * k * excess * excess;
                }
            }
        }
        c
    };

    // Tightest admissible heuristic scale: the lowest possible cost of
    // covering one metre of great-circle distance. The multiplicative
    // bonuses can price an edge slightly below its physical length
    // (designated cycleway on a signed network ≈ 0.97×); every other term
    // (α-deviation, turn, descent, ascent) only adds cost. Computed from the
    // actual multiplier tables so future tuning can't silently break
    // admissibility — an overestimating heuristic would cost optimality,
    // while needless slack inflates the search quadratically (the old
    // hardcoded 0.5 roughly doubled the explored radius for bike routing).
    let h_scale: f64 = if bike_active || mtb_active {
        let min_class = (0..=BCLASS_MASK)
            .map(&bike_class_mult)
            .fold(f64::INFINITY, f64::min);
        let min_surface = (0..=SURFACE_FERRY)
            .map(&surface_mult)
            .fold(f64::INFINITY, f64::min);
        (min_class * min_surface * DESIGNATED_BONUS * NETWORK_BONUS).min(1.0)
    } else {
        1.0
    };
    let elev_at = |n: NodeIndex| -> f32 {
        elevations
            .and_then(|el| el.get(n.index()).copied())
            .unwrap_or(f32::NAN)
    };
    // Incoming bearing for the turn penalty when leaving `at` having arrived
    // from `before`; None without a predecessor, for foot-only routing, or
    // after a zero-length segment.
    let bearing_in = |before: Option<NodeIndex>, at: LatLon| -> Option<f64> {
        if !turn_active {
            return None;
        }
        let pp = g[before?];
        (pp != at).then(|| fast_bearing_rad(pp, at))
    };
    // A* only stops at nodes where the route can branch. Mid-road shape
    // points (two edges, two distinct neighbours) leave one way forward —
    // U-turns are forbidden — so `walk` crosses them in one go: from `from`
    // (reached via `before`) along `first` until a junction, the start or the
    // end. It charges every edge exactly what the per-node search did (the
    // cost model, turn penalty against the previous node, midpoint
    // deviation as the mean of endpoint deviations), and returns the node it
    // stopped at, the walk's cost, and the node just before it. None when an
    // edge on the way is forbidden (mode, surface, corridor, ∞ cost), which
    // makes the whole chain impassable — as it was node by node.
    let max_steps = g.node_count();
    let walk = |from: NodeIndex,
                before: Option<NodeIndex>,
                first: EdgeReference<'_, EdgeData>|
     -> Option<(NodeIndex, f64, NodeIndex)> {
        let (mut u, mut before, mut er) = (from, before, first);
        let mut pu = g[u];
        let mut dev_u = xt.signed(pu);
        let mut z_u = elev_at(u);
        let mut total = 0.0;
        for _ in 0..max_steps {
            let e = er.weight();
            // Cheapest rejections first: bitmask checks need no geometry.
            if (e.modes & modes) == 0 || (paved_only && e.surface != SURFACE_PAVED) {
                return None;
            }
            let v = er.target();
            let pv = g[v];
            let dev_v = xt.signed(pv);
            let dev = ((dev_u + dev_v) * 0.5).abs();
            if dev > corridor_max {
                return None;
            }
            let z_v = elev_at(v);
            let c = cost(
                e,
                dev,
                bearing_in(before, pu),
                pu,
                pv,
                elevations.map(|_| (z_u, z_v)),
            );
            if !c.is_finite() {
                return None;
            }
            total += c;
            if v == end || v == start || graph.is_junction(v) {
                return Some((v, total, u));
            }
            // Shape point: the way on is the edge not leading back to `u`.
            er = g.edges(v).find(|r| r.target() != u)?;
            (before, u, pu, dev_u, z_u) = (Some(u), v, pv, dev_v, z_v);
        }
        None
    };

    // A* over the stop nodes, with FxHash-backed state and lazy deletion.
    // The turn penalty uses each node's *current* predecessor at expansion
    // time rather than augmenting the state with it — a greedy approximation
    // (a predecessor may later be revised by a cheaper path), but the
    // heuristic-driven search converges quickly on the canonical one and
    // full (node, incoming edge) states would multiply the state space ~10×
    // at marginal quality gain. `start_prev` is a virtual predecessor of
    // `start` (set by `route_loop`) so the turn penalty and U-turn rule
    // extend across leg boundaries.
    let h_of = |n: NodeIndex| heur.distance(g[n]) * h_scale;
    let mut state: FxHashMap<NodeIndex, NodeState> =
        FxHashMap::with_capacity_and_hasher(1 << 10, Default::default());
    let mut open: BinaryHeap<Frontier> = BinaryHeap::with_capacity(1 << 10);
    let h_start = h_of(start);
    state.insert(
        start,
        NodeState {
            g: 0.0,
            h: h_start,
            from: start,
            via: None,
            prev: start_prev,
        },
    );
    open.push(Frontier {
        f: h_start,
        g: 0.0,
        node: start,
    });
    let mut found = false;
    while let Some(Frontier {
        g: popped_g, node, ..
    }) = open.pop()
    {
        let (cur_g, prev) = match state.get(&node) {
            Some(st) => (st.g, st.prev),
            None => continue,
        };
        // Lazy deletion: a fresher, cheaper entry for this node was pushed
        // after this one.
        if popped_g > cur_g {
            continue;
        }
        if node == end {
            found = true;
            break;
        }
        for er in g.edges(node) {
            // No immediate U-turn back to the predecessor; keeps the path
            // topologically simple, across leg boundaries via `start_prev`.
            if Some(er.target()) == prev {
                continue;
            }
            let Some((next, c, before)) = walk(node, prev, er) else {
                continue;
            };
            let tentative = cur_g + c;
            match state.entry(next) {
                std::collections::hash_map::Entry::Occupied(mut oe) => {
                    let st = oe.get_mut();
                    if tentative < st.g {
                        st.g = tentative;
                        st.from = node;
                        st.via = Some(er.id());
                        st.prev = Some(before);
                        open.push(Frontier {
                            f: tentative + st.h,
                            g: tentative,
                            node: next,
                        });
                    }
                }
                std::collections::hash_map::Entry::Vacant(ve) => {
                    let h = h_of(next);
                    ve.insert(NodeState {
                        g: tentative,
                        h,
                        from: node,
                        via: Some(er.id()),
                        prev: Some(before),
                    });
                    open.push(Frontier {
                        f: tentative + h,
                        g: tentative,
                        node: next,
                    });
                }
            }
        }
    }
    if !found {
        return Err(anyhow!(
            "no route found between selected nodes — try a wider corridor or different modes"
        ));
    }

    // Rebuild the path backwards, one walk at a time: each stop node records
    // the walk (origin + first edge) that last improved it, and re-walking it
    // is deterministic. Exact edges are known, so parallel edges need no
    // re-selection.
    let mut walks: Vec<(Vec<NodeIndex>, Vec<EdgeData>)> = Vec::new();
    let mut cur = end;
    while cur != start {
        let st = &state[&cur];
        let mut seg_nodes = Vec::new();
        let mut seg_edges = Vec::new();
        let (mut u, mut e) = (st.from, st.via.expect("non-start state has a walk"));
        loop {
            let (a, b) = g.edge_endpoints(e).expect("edge in graph");
            let v = if a == u { b } else { a };
            seg_edges.push(g[e]);
            seg_nodes.push(v);
            if v == cur {
                break;
            }
            e = g
                .edges(v)
                .find(|r| r.target() != u)
                .expect("shape point has a way on")
                .id();
            u = v;
        }
        walks.push((seg_nodes, seg_edges));
        cur = st.from;
    }
    let mut node_indices: Vec<NodeIndex> = vec![start];
    let mut edges: Vec<EdgeData> = Vec::new();
    for (seg_nodes, seg_edges) in walks.into_iter().rev() {
        node_indices.extend(seg_nodes);
        edges.extend(seg_edges);
    }
    let mut points: Vec<LatLon> = node_indices.iter().map(|&n| g[n]).collect();
    // Splice out any graph-level loops the A* expansion may have left
    // behind. Single-leg A* normally can't revisit a node, but the cross-leg
    // path concatenation in `route_loop` can; this pass also guards single
    // legs against any future change that might allow revisits.
    let _pruned = prune_revisits(&mut node_indices, &mut points, &mut edges);
    // Geometric loop pruning catches the "out-and-back via a parallel
    // cycleway / service road" pattern: the polyline returns within 30 m of
    // an earlier point after a 250 m+ excursion. Single-leg A* can produce
    // these (different graph nodes, so the graph-revisit pass above misses
    // them) and they are exactly the "weird tails" the user flagged.
    if bike_active || mtb_active {
        let _g = prune_geometric_loops(&mut node_indices, &mut points, &mut edges, 60.0, 200.0);
    }
    let total_length: f64 = edges.iter().map(|e| e.length_m).sum();

    Ok(RouteResult {
        points,
        edges,
        total_length_m: total_length,
        node_indices,
    })
}

/// Per-leg failure detail returned by `route_loop` so the caller can report
/// which leg of the shape is unroutable rather than a generic error.
#[derive(Debug)]
pub struct LegFailure {
    pub leg_index: usize,
    pub from: LatLon,
    pub to: LatLon,
    pub message: String,
}

/// Route a closed loop that visits the given vertices in order, returning a
/// single concatenated path. Each leg is routed independently with the same
/// `params` (mode mask, corridor cap, paved-only, α). Adjacent legs share a
/// junction node; the duplicate is dropped from the concatenation so total
/// length and edge counts stay correct.
pub fn route_loop(
    graph: &Graph,
    vertices: &[LatLon],
    params: &RouteParams,
) -> Result<RouteResult, LegFailure> {
    if vertices.len() < 2 {
        return Err(LegFailure {
            leg_index: 0,
            from: vertices.first().copied().unwrap_or(LatLon::new(0.0, 0.0)),
            to: vertices.last().copied().unwrap_or(LatLon::new(0.0, 0.0)),
            message: "shape has no legs".into(),
        });
    }
    let mut all_points: Vec<LatLon> = Vec::new();
    let mut all_edges: Vec<EdgeData> = Vec::new();
    let mut all_nodes: Vec<NodeIndex> = Vec::new();
    // The predecessor of the *next* leg's start is the second-to-last node
    // of the current leg. Threading it through `shortest_with_start_prev`
    // makes A* forbid going right back through that node — kills the
    // hairpin "tail" pattern at shape vertices.
    let mut start_prev: Option<NodeIndex> = None;
    for (i, w) in vertices.windows(2).enumerate() {
        let pair = graph
            .closest_connected_pair(w[0], w[1], params.modes, params.paved_only)
            .ok_or(LegFailure {
                leg_index: i,
                from: w[0],
                to: w[1],
                message: "no usable road near the leg endpoints".into(),
            })?;
        let (s_idx, e_idx, _, _) = pair;
        // Only thread `start_prev` when the next leg's start coincides with
        // the previous leg's end node — otherwise the predecessor refers to
        // a different node and would mis-guide the turn penalty.
        let prev_to_pass = match (all_nodes.last(), start_prev) {
            (Some(&last), Some(_)) if last == s_idx => start_prev,
            _ => None,
        };
        // Threading can stall A* if the start node is a degree-1 dead end
        // whose only neighbour is the forbidden predecessor. Fall back to
        // unthreaded routing in that case so the loop completes — turn
        // smoothing is sacrificed at the junction, but a route exists.
        let leg =
            match shortest_with_start_prev(graph, s_idx, e_idx, prev_to_pass, w[0], w[1], params) {
                Ok(r) => r,
                Err(_) if prev_to_pass.is_some() => {
                    shortest_with_start_prev(graph, s_idx, e_idx, None, w[0], w[1], params)
                        .map_err(|e| LegFailure {
                            leg_index: i,
                            from: w[0],
                            to: w[1],
                            message: e.to_string(),
                        })?
                }
                Err(e) => {
                    return Err(LegFailure {
                        leg_index: i,
                        from: w[0],
                        to: w[1],
                        message: e.to_string(),
                    });
                }
            };
        if all_points.is_empty() {
            all_points.extend(&leg.points);
            all_nodes.extend(&leg.node_indices);
        } else {
            // Skip the first point/node of subsequent legs to avoid a
            // duplicate vertex at the junction with the previous leg.
            all_points.extend(&leg.points[1..]);
            all_nodes.extend(&leg.node_indices[1..]);
        }
        all_edges.extend(&leg.edges);
        // Predecessor of next leg's start = second-to-last node of this leg.
        start_prev = if leg.node_indices.len() >= 2 {
            Some(leg.node_indices[leg.node_indices.len() - 2])
        } else {
            None
        };
    }
    // Cross-leg graph-revisit pruning was tried and reverted (it collapsed
    // legitimate shape cross-overs whose two legs deliberately pass through
    // the same major junction). The *geometric* pruner is safer: it only
    // fires on near-revisits within 30 m, which a legitimate cross-over at
    // a major intersection would clear by far. Bike/mtb only — turn smoothing
    // and tails are concerns specific to cycling, not foot routing.
    let bike_or_mtb = params.modes & (crate::osm::MODE_BIKE | crate::osm::MODE_MTB) != 0;
    if bike_or_mtb {
        prune_geometric_loops(&mut all_nodes, &mut all_points, &mut all_edges, 60.0, 200.0);
    }
    let total_length: f64 = all_edges.iter().map(|e| e.length_m).sum();
    Ok(RouteResult {
        points: all_points,
        edges: all_edges,
        total_length_m: total_length,
        node_indices: all_nodes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::haversine;
    use crate::osm::{
        BCLASS_CYCLEWAY, BCLASS_PATH, BCLASS_PRIMARY, BCLASS_RESIDENTIAL, MODE_BIKE, MODE_FOOT,
        MODE_MTB, SURFACE_PATH, SURFACE_PAVED,
    };
    use petgraph::graph::UnGraph;

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
                    bike_attrs: BCLASS_RESIDENTIAL,
                },
            );
        };
        edge(&mut g, a, m1);
        edge(&mut g, m1, m2);
        edge(&mut g, m2, b);
        Graph::from_graph(g)
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

    /// Wrap a bare UnGraph in a `Graph` with its derived indices
    /// (components, R-tree, junctions), as a real build would.
    fn wrap_graph(g: UnGraph<LatLon, EdgeData>) -> Graph {
        Graph::from_graph(g)
    }

    fn add_edge(
        g: &mut UnGraph<LatLon, EdgeData>,
        x: NodeIndex,
        y: NodeIndex,
        bike_attrs: u8,
    ) -> petgraph::graph::EdgeIndex {
        add_edge_with_surface(g, x, y, bike_attrs, SURFACE_PAVED)
    }

    fn add_edge_with_surface(
        g: &mut UnGraph<LatLon, EdgeData>,
        x: NodeIndex,
        y: NodeIndex,
        bike_attrs: u8,
        surface: u8,
    ) -> petgraph::graph::EdgeIndex {
        add_edge_with_modes(
            g,
            x,
            y,
            bike_attrs,
            surface,
            MODE_FOOT | MODE_BIKE | MODE_MTB,
        )
    }

    fn add_edge_with_modes(
        g: &mut UnGraph<LatLon, EdgeData>,
        x: NodeIndex,
        y: NodeIndex,
        bike_attrs: u8,
        surface: u8,
        modes: u8,
    ) -> petgraph::graph::EdgeIndex {
        let length = haversine(g[x], g[y]);
        g.add_edge(
            x,
            y,
            EdgeData {
                length_m: length,
                modes,
                surface,
                bike_attrs,
            },
        )
    }

    fn bike_params(half_width_m: f64) -> RouteParams {
        RouteParams {
            modes: MODE_BIKE,
            half_width_m,
            alpha: 0.0,
            corridor_max_m: half_width_m,
            paved_only: false,
        }
    }

    #[test]
    fn turn_penalty_prefers_smoother_route_over_shorter() {
        // Two A→C options on a synthetic graph:
        //   smooth: A → B → C  (straight along the equator, ~222 m, no bend)
        //   sharp:  A → D → C  (a V-shape with ~90° bend at D, ~156 m total)
        // Without the turn penalty the V wins on raw length. With TURN_PEN=100
        // m·rad the 90° pivot adds ~157 m, so the smooth route must win.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.001));
        let c = g.add_node(LatLon::new(0.0, 0.002));
        let d = g.add_node(LatLon::new(0.0005, 0.001));
        add_edge(&mut g, a, b, BCLASS_RESIDENTIAL);
        add_edge(&mut g, b, c, BCLASS_RESIDENTIAL);
        add_edge(&mut g, a, d, BCLASS_RESIDENTIAL);
        add_edge(&mut g, d, c, BCLASS_RESIDENTIAL);
        let graph = wrap_graph(g);
        let params = bike_params(100_000.0);
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(2),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(r.points.len(), 3, "expected 3-point straight route");
        assert!(
            r.points[1].lat.abs() < 1e-9,
            "middle vertex should be B (lat=0), got {:?}",
            r.points[1]
        );
    }

    #[test]
    fn foot_mode_ignores_bike_class_preference() {
        // The cycleway-vs-primary split that `bike_prefers_cycleway_over_primary`
        // exercises must NOT influence foot routing — class multipliers and
        // turn penalty are bike/mtb-only.  With both branches identical in
        // length and surface, foot mode is free to pick either.  We just
        // assert it produces a valid route of the expected length.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.002));
        let p_mid = g.add_node(LatLon::new(0.0005, 0.001));
        let c_mid = g.add_node(LatLon::new(-0.0005, 0.001));
        add_edge(&mut g, a, p_mid, BCLASS_PRIMARY);
        add_edge(&mut g, p_mid, b, BCLASS_PRIMARY);
        add_edge(&mut g, a, c_mid, BCLASS_CYCLEWAY);
        add_edge(&mut g, c_mid, b, BCLASS_CYCLEWAY);
        let graph = wrap_graph(g);
        let mut params = bike_params(100_000.0);
        params.modes = MODE_FOOT;
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(r.points.len(), 3, "expected 3-point route A→mid→B");
    }

    #[test]
    fn bike_prefers_cycleway_over_primary_at_equal_length() {
        // Two symmetric parallel routes A→B of identical length and turn
        // geometry, differing only in highway class. Bike mode must pick
        // the cycleway branch over the primary branch.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.002));
        let p_mid = g.add_node(LatLon::new(0.0005, 0.001)); // primary detour (north)
        let c_mid = g.add_node(LatLon::new(-0.0005, 0.001)); // cycleway detour (south)
        add_edge(&mut g, a, p_mid, BCLASS_PRIMARY);
        add_edge(&mut g, p_mid, b, BCLASS_PRIMARY);
        add_edge(&mut g, a, c_mid, BCLASS_CYCLEWAY);
        add_edge(&mut g, c_mid, b, BCLASS_CYCLEWAY);
        let graph = wrap_graph(g);
        let params = bike_params(100_000.0);
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert!(
            r.points[1].lat < 0.0,
            "bike route should travel via cycleway midpoint (lat<0), got {:?}",
            r.points[1]
        );
    }

    #[test]
    fn road_bike_prefers_paved_detour_over_short_path() {
        // Direct A→B is a rough path (~222 m). The paved residential detour is
        // ~314 m, but should still win for pure bike mode because road bikes
        // should not treat MTB/foot paths as cheap shortcuts.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.002));
        let c = g.add_node(LatLon::new(0.001, 0.001));
        add_edge_with_surface(&mut g, a, b, BCLASS_PATH, SURFACE_PATH);
        add_edge(&mut g, a, c, BCLASS_RESIDENTIAL);
        add_edge(&mut g, c, b, BCLASS_RESIDENTIAL);
        let graph = wrap_graph(g);
        let params = bike_params(100_000.0);
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(r.points.len(), 3, "bike should take paved detour");
        assert!(
            r.points[1].lat > 0.0,
            "middle vertex should be paved detour, got {:?}",
            r.points[1]
        );
    }

    #[test]
    fn mtb_mode_still_accepts_short_path() {
        // Same graph as `road_bike_prefers_paved_detour_over_short_path`, but
        // MTB mode keeps path costs low enough that the shorter direct leg is
        // acceptable.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.002));
        let c = g.add_node(LatLon::new(0.001, 0.001));
        add_edge_with_surface(&mut g, a, b, BCLASS_PATH, SURFACE_PATH);
        add_edge(&mut g, a, c, BCLASS_RESIDENTIAL);
        add_edge(&mut g, c, b, BCLASS_RESIDENTIAL);
        let graph = wrap_graph(g);
        let mut params = bike_params(100_000.0);
        params.modes = MODE_MTB;
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(r.points.len(), 2, "mtb should accept short path");
    }

    #[test]
    fn bike_plus_foot_prefers_rideable_detour_over_foot_only_path() {
        // With foot+bike enabled, a direct foot-only path is technically
        // allowed. For road-bike-like routing it should still lose to a
        // reasonable paved bike detour.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.002));
        let c = g.add_node(LatLon::new(0.001, 0.001));
        add_edge_with_modes(&mut g, a, b, BCLASS_PATH, SURFACE_PATH, MODE_FOOT);
        add_edge(&mut g, a, c, BCLASS_RESIDENTIAL);
        add_edge(&mut g, c, b, BCLASS_RESIDENTIAL);
        let graph = wrap_graph(g);
        let mut params = bike_params(100_000.0);
        params.modes = MODE_FOOT | MODE_BIKE;
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(
            r.points.len(),
            3,
            "foot+bike should avoid foot-only shortcut"
        );
    }

    #[test]
    fn ascent_penalty_prefers_flat_over_steep() {
        // Two A→B options of comparable length. The "steep" branch climbs
        // ~10 m over a 55 m edge (~18 % grade) → the hinged ascent term
        // makes it expensive. The "flat" branch has equal elevation across
        // both vertices and should win.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.001));
        let flat = g.add_node(LatLon::new(0.0005, 0.0005));
        let steep = g.add_node(LatLon::new(-0.0005, 0.0005));
        add_edge(&mut g, a, flat, BCLASS_RESIDENTIAL);
        add_edge(&mut g, flat, b, BCLASS_RESIDENTIAL);
        add_edge(&mut g, a, steep, BCLASS_RESIDENTIAL);
        add_edge(&mut g, steep, b, BCLASS_RESIDENTIAL);
        let mut graph = wrap_graph(g);
        graph.elevations_m = Some(vec![100.0, 100.0, 100.0, 110.0]);
        let params = bike_params(100_000.0);
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(1),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.001),
            &params,
        )
        .expect("route");
        assert!(
            r.points[1].lat > 0.0,
            "bike route should take the flat branch (lat>0), got {:?}",
            r.points[1]
        );
    }

    #[test]
    fn prune_revisits_drops_simple_loop() {
        // Synthetic path: 0 → 1 → 2 → 3 → 1 → 4.  The cycle "1 → 2 → 3 → 1"
        // returns to node 1; the splice should collapse it to "0 → 1 → 4".
        let ni = |i: u32| NodeIndex::from(i);
        let mut nodes = vec![ni(0), ni(1), ni(2), ni(3), ni(1), ni(4)];
        let mut points = vec![
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 1.0),
            LatLon::new(0.0, 2.0),
            LatLon::new(0.0, 3.0),
            LatLon::new(0.0, 1.0),
            LatLon::new(0.0, 4.0),
        ];
        let dummy = EdgeData {
            length_m: 100.0,
            modes: MODE_BIKE,
            surface: SURFACE_PAVED,
            bike_attrs: BCLASS_RESIDENTIAL,
        };
        let mut edges = vec![dummy; 5];
        let removed = super::prune_revisits(&mut nodes, &mut points, &mut edges);
        assert_eq!(removed, 3);
        assert_eq!(nodes, vec![ni(0), ni(1), ni(4)]);
        assert_eq!(points.len(), 3);
        assert_eq!(edges.len(), 2);
    }

    #[test]
    fn prune_revisits_keeps_clean_paths_intact() {
        let ni = |i: u32| NodeIndex::from(i);
        let mut nodes = vec![ni(0), ni(1), ni(2), ni(3)];
        let mut points = vec![
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 1.0),
            LatLon::new(0.0, 2.0),
            LatLon::new(0.0, 3.0),
        ];
        let dummy = EdgeData {
            length_m: 100.0,
            modes: MODE_BIKE,
            surface: SURFACE_PAVED,
            bike_attrs: BCLASS_RESIDENTIAL,
        };
        let mut edges = vec![dummy; 3];
        let removed = super::prune_revisits(&mut nodes, &mut points, &mut edges);
        assert_eq!(removed, 0);
        assert_eq!(nodes.len(), 4);
        assert_eq!(edges.len(), 3);
    }

    #[test]
    fn prune_geometric_loops_collapses_a_lobe() {
        // Build a 21-point polyline: 10 points heading east, 10 points
        // making a perpendicular lobe (going north 200m then south 200m
        // back), then continuing east. The lobe tip is at the apex; the
        // return point sits ~10 m from where the lobe started, so the
        // pruner should drop the lobe's interior.
        use petgraph::graph::NodeIndex;
        let ni = |i: u32| NodeIndex::from(i);
        let mut nodes: Vec<NodeIndex> = (0..21).map(ni).collect();
        // Points: 0..=4 east along y=0, lat increasing.
        // 5..=14 the lobe (going north to peak then back).
        // 15..=20 east continuation.
        let mut points: Vec<LatLon> = Vec::with_capacity(21);
        for i in 0..5 {
            points.push(LatLon::new(0.0, 0.0005 * i as f64));
        }
        // Lobe: enters from (0, 0.002), goes north to (0.002, 0.002) then back.
        let entry = LatLon::new(0.0, 0.002);
        for k in 0..10 {
            let theta = (k as f64) * std::f64::consts::PI / 9.0;
            let lat = entry.lat + 0.002 * theta.sin() * 0.5;
            let lon = entry.lon + 0.00001 * k as f64;
            points.push(LatLon::new(lat, lon));
        }
        // Continue east, starting ~10 m from the lobe entry (the return point).
        for i in 0..6 {
            points.push(LatLon::new(0.0, 0.0021 + 0.0005 * i as f64));
        }
        let dummy = EdgeData {
            length_m: 50.0,
            modes: MODE_BIKE,
            surface: SURFACE_PAVED,
            bike_attrs: BCLASS_RESIDENTIAL,
        };
        let mut edges = vec![dummy; 20];
        let removed =
            super::prune_geometric_loops(&mut nodes, &mut points, &mut edges, 30.0, 100.0);
        assert!(removed > 0, "lobe should have been spliced");
        assert!(points.len() < 21);
        assert_eq!(
            edges.len() + 1,
            nodes.len(),
            "node/edge counts must stay aligned after splice"
        );
    }

    #[test]
    fn prune_geometric_loops_leaves_clean_paths_alone() {
        // Straight polyline with no self-proximity → nothing to splice.
        let ni = |i: u32| petgraph::graph::NodeIndex::from(i);
        let mut nodes: Vec<_> = (0..30).map(ni).collect();
        let mut points: Vec<LatLon> = (0..30)
            .map(|i| LatLon::new(0.0, 0.001 * i as f64))
            .collect();
        let dummy = EdgeData {
            length_m: 100.0,
            modes: MODE_BIKE,
            surface: SURFACE_PAVED,
            bike_attrs: BCLASS_RESIDENTIAL,
        };
        let mut edges = vec![dummy; 29];
        let removed =
            super::prune_geometric_loops(&mut nodes, &mut points, &mut edges, 30.0, 250.0);
        assert_eq!(removed, 0);
        assert_eq!(points.len(), 30);
        assert_eq!(edges.len(), 29);
    }

    #[test]
    fn prune_revisits_collapses_nested_loops() {
        // 0 → 1 → 2 → 1 → 3 → 1 → 4: two separate returns to 1.  After the
        // first splice, 1 is at index 1; the second 1 (originally idx 5)
        // becomes another revisit and gets collapsed too.  Final path:
        // 0 → 1 → 4.
        let ni = |i: u32| NodeIndex::from(i);
        let mut nodes = vec![ni(0), ni(1), ni(2), ni(1), ni(3), ni(1), ni(4)];
        let mut points = vec![LatLon::new(0.0, 0.0); nodes.len()];
        let dummy = EdgeData {
            length_m: 100.0,
            modes: MODE_BIKE,
            surface: SURFACE_PAVED,
            bike_attrs: BCLASS_RESIDENTIAL,
        };
        let mut edges = vec![dummy; nodes.len() - 1];
        let removed = super::prune_revisits(&mut nodes, &mut points, &mut edges);
        assert_eq!(removed, 4);
        assert_eq!(nodes, vec![ni(0), ni(1), ni(4)]);
        assert_eq!(edges.len(), 2);
    }

    #[test]
    fn u_turn_back_to_predecessor_is_skipped() {
        // A — B — C linear chain. After A* arrives at B with prev=A, the
        // edge B→A must not be considered (would be a U-turn). Routing
        // A→C therefore visits each node exactly once.
        let mut g = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(0.0, 0.0));
        let b = g.add_node(LatLon::new(0.0, 0.001));
        let c = g.add_node(LatLon::new(0.0, 0.002));
        add_edge(&mut g, a, b, BCLASS_RESIDENTIAL);
        add_edge(&mut g, b, c, BCLASS_RESIDENTIAL);
        let graph = wrap_graph(g);
        let params = bike_params(100_000.0);
        let r = shortest(
            &graph,
            NodeIndex::new(0),
            NodeIndex::new(2),
            LatLon::new(0.0, 0.0),
            LatLon::new(0.0, 0.002),
            &params,
        )
        .expect("route");
        assert_eq!(r.points.len(), 3);
        assert_eq!(r.points[0], LatLon::new(0.0, 0.0));
        assert_eq!(r.points[1], LatLon::new(0.0, 0.001));
        assert_eq!(r.points[2], LatLon::new(0.0, 0.002));
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

    /// A road network for randomized checks: a grid of junctions whose
    /// roads are winding chains of shape points, plus dead-end spurs, loop
    /// roads, parallel roads, and random modes / surfaces / classes.
    fn random_network(rng: &mut rand::rngs::StdRng) -> UnGraph<LatLon, EdgeData> {
        use rand::Rng;
        fn jitter(rng: &mut rand::rngs::StdRng) -> f64 {
            rng.gen_range(-0.0004..0.0004)
        }
        fn random_edge(rng: &mut rand::rngs::StdRng, length_m: f64) -> EdgeData {
            let modes = match rng.gen_range(0..10) {
                0 => MODE_FOOT,
                1 => MODE_BIKE,
                2 => MODE_FOOT | MODE_MTB,
                _ => MODE_FOOT | MODE_BIKE | MODE_MTB,
            };
            let mut bike_attrs = rng.gen_range(0..=9u8);
            if rng.gen_bool(0.2) {
                bike_attrs |= crate::osm::BATTR_DESIGNATED;
            }
            if rng.gen_bool(0.2) {
                bike_attrs |= crate::osm::BATTR_CYCLE_NETWORK;
            }
            EdgeData {
                length_m,
                modes,
                surface: rng.gen_range(0..=5u8),
                bike_attrs,
            }
        }
        /// A road from `a` to `b` through `k` jittered shape points.
        fn road(
            g: &mut UnGraph<LatLon, EdgeData>,
            rng: &mut rand::rngs::StdRng,
            a: NodeIndex,
            b: NodeIndex,
            k: usize,
        ) {
            let (pa, pb) = (g[a], g[b]);
            let mut prev = a;
            for i in 1..=k {
                let t = i as f64 / (k + 1) as f64;
                let p = LatLon::new(
                    pa.lat + (pb.lat - pa.lat) * t + jitter(rng),
                    pa.lon + (pb.lon - pa.lon) * t + jitter(rng),
                );
                let n = g.add_node(p);
                let e = random_edge(rng, haversine(g[prev], p));
                g.add_edge(prev, n, e);
                prev = n;
            }
            let e = random_edge(rng, haversine(g[prev], pb));
            g.add_edge(prev, b, e);
        }

        let mut g: UnGraph<LatLon, EdgeData> = UnGraph::new_undirected();
        let side = rng.gen_range(3..=6);
        let spacing = 0.003; // ~300 m
        let grid: Vec<NodeIndex> = (0..side * side)
            .map(|i| {
                let (r, c) = ((i / side) as f64, (i % side) as f64);
                let p = LatLon::new(
                    46.0 + r * spacing + jitter(rng),
                    7.0 + c * spacing + jitter(rng),
                );
                g.add_node(p)
            })
            .collect();
        for r in 0..side {
            for c in 0..side {
                let here = grid[r * side + c];
                for (dr, dc) in [(0, 1), (1, 0), (1, 1)] {
                    let (nr, nc) = (r + dr, c + dc);
                    let skip = if dr + dc == 2 { 0.8 } else { 0.15 };
                    if nr >= side || nc >= side || rng.gen_bool(skip) {
                        continue;
                    }
                    let there = grid[nr * side + nc];
                    let k = rng.gen_range(0..=6);
                    road(&mut g, rng, here, there, k);
                    if rng.gen_bool(0.05) {
                        road(&mut g, rng, here, there, 0); // parallel road
                    }
                }
                let p = g[here];
                if rng.gen_bool(0.15) {
                    let tip = g.add_node(LatLon::new(p.lat + jitter(rng), p.lon + jitter(rng)));
                    let k = rng.gen_range(0..=3);
                    road(&mut g, rng, here, tip, k); // dead-end spur
                }
                if rng.gen_bool(0.08) {
                    // Loop road leaving and re-entering the same junction.
                    let far = g.add_node(LatLon::new(p.lat + 0.0008, p.lon + 0.0008));
                    let (k1, k2) = (rng.gen_range(1..=3), rng.gen_range(1..=3));
                    road(&mut g, rng, here, far, k1);
                    road(&mut g, rng, far, here, k2);
                }
            }
        }
        g
    }

    /// The chain-walking search must return exactly what a per-node search
    /// returns. Marking every node a junction degenerates `walk` into single
    /// edges — the classic per-node A* — so both run the same cost model and
    /// differ only in how they cross shape points.
    #[test]
    fn chain_walking_matches_per_node_search() {
        use rand::{rngs::StdRng, Rng, SeedableRng};
        let mut rng = StdRng::seed_from_u64(0x5eed);
        let (mut routed, mut no_route, mut mid_chain) = (0, 0, 0);
        for case in 0..400 {
            let net = random_network(&mut rng);
            let mut chains = Graph::from_graph(net.clone());
            let mut per_node = Graph::from_graph(net);
            per_node.junctions.iter_mut().for_each(|w| *w = u64::MAX);
            if rng.gen_bool(0.5) {
                let mut z = 500.0_f32;
                let elev: Vec<f32> = (0..chains.graph.node_count())
                    .map(|_| {
                        z += rng.gen_range(-15.0..15.0);
                        z
                    })
                    .collect();
                chains.elevations_m = Some(elev.clone());
                per_node.elevations_m = Some(elev);
            }
            let n = chains.graph.node_count();
            let start = NodeIndex::new(rng.gen_range(0..n));
            let end = NodeIndex::new(rng.gen_range(0..n));
            let start_prev = if rng.gen_bool(0.4) {
                chains.graph.neighbors(start).next()
            } else {
                None
            };
            let modes = [
                MODE_FOOT,
                MODE_BIKE,
                MODE_FOOT | MODE_BIKE,
                MODE_BIKE | MODE_MTB,
                MODE_FOOT | MODE_BIKE | MODE_MTB,
            ][rng.gen_range(0..5)];
            let half = rng.gen_range(150.0..2500.0);
            let params = RouteParams {
                modes,
                half_width_m: half,
                alpha: rng.gen_range(0.0..12.0),
                corridor_max_m: half,
                paved_only: rng.gen_bool(0.1),
            };
            let (a, b) = (chains.graph[start], chains.graph[end]);
            let fast = shortest_with_start_prev(&chains, start, end, start_prev, a, b, &params);
            let slow = shortest_with_start_prev(&per_node, start, end, start_prev, a, b, &params);
            match (fast, slow) {
                (Ok(f), Ok(s)) => {
                    assert_eq!(
                        f.node_indices, s.node_indices,
                        "case {case}: different paths"
                    );
                    let key = |r: &RouteResult| {
                        r.edges
                            .iter()
                            .map(|e| (e.length_m.to_bits(), e.modes, e.surface, e.bike_attrs))
                            .collect::<Vec<_>>()
                    };
                    assert_eq!(key(&f), key(&s), "case {case}: different edges");
                    routed += 1;
                    if !chains.is_junction(start) || !chains.is_junction(end) {
                        mid_chain += 1;
                    }
                }
                (Err(_), Err(_)) => no_route += 1,
                (f, s) => panic!(
                    "case {case}: chain walk {:?} vs per-node {:?}",
                    f.map(|r| r.node_indices.len()),
                    s.map(|r| r.node_indices.len())
                ),
            }
        }
        // Make sure the comparison exercised the interesting cases.
        println!("{routed} routed ({mid_chain} from/to mid-chain), {no_route} without a route");
        assert!(
            routed > 150 && no_route > 20,
            "routed {routed}, no route {no_route}"
        );
        assert!(
            mid_chain > 100,
            "only {mid_chain} routes started/ended mid-chain"
        );
    }
}
