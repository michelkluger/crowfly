//! Read an OSM PBF and build a corridor-bounded routing graph.

use crate::geodesy::{haversine, LatLon};
use anyhow::{Context, Result};
use indicatif::{ProgressBar, ProgressStyle};
use osmpbfreader::{OsmObj, OsmPbfReader};
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::unionfind::UnionFind;
use petgraph::visit::EdgeRef;
use rustc_hash::{FxHashMap, FxHashSet};
use std::fs::File;
use std::path::Path;

pub const MODE_FOOT: u8 = 1 << 0;
pub const MODE_BIKE: u8 = 1 << 1;
pub const MODE_MTB: u8 = 1 << 2;
pub const MODE_FERRY: u8 = 1 << 3;

/// Coarse surface categories useful for route comparison. Stored as u8 in
/// EdgeData so it fits next to the bitmask without alignment waste.
pub const SURFACE_PAVED: u8 = 0;
pub const SURFACE_GRAVEL: u8 = 1;
pub const SURFACE_PATH: u8 = 2;
pub const SURFACE_UNPAVED: u8 = 3;
pub const SURFACE_OTHER: u8 = 4;
pub const SURFACE_FERRY: u8 = 5;
pub const SURFACE_LABELS: [&str; 6] =
    ["paved", "gravel", "path", "unpaved", "other", "ferry"];

#[derive(Copy, Clone, Debug)]
pub struct EdgeData {
    pub length_m: f64,
    pub modes: u8,
    pub surface: u8,
    pub way_id: i64,
}

#[derive(Copy, Clone, Debug)]
pub struct BBox {
    pub lat_min: f64,
    pub lat_max: f64,
    pub lon_min: f64,
    pub lon_max: f64,
}

impl BBox {
    pub fn padded(&self, deg: f64) -> Self {
        Self {
            lat_min: self.lat_min - deg,
            lat_max: self.lat_max + deg,
            lon_min: self.lon_min - deg,
            lon_max: self.lon_max + deg,
        }
    }
    pub fn contains(&self, p: LatLon) -> bool {
        p.lat >= self.lat_min
            && p.lat <= self.lat_max
            && p.lon >= self.lon_min
            && p.lon <= self.lon_max
    }
}

/// One entry in the spatial R-tree. Stores its coordinates *and* the
/// NodeIndex so we can recover the graph node from a nearest-neighbour query.
#[derive(Clone, Copy, Debug)]
pub struct NodePoint {
    pub idx: NodeIndex,
    pub lon: f64,
    pub lat: f64,
}

impl rstar::RTreeObject for NodePoint {
    type Envelope = rstar::AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_point([self.lon, self.lat])
    }
}

impl rstar::PointDistance for NodePoint {
    fn distance_2(&self, p: &[f64; 2]) -> f64 {
        // Squared planar distance in lon/lat. Not metres, but monotonic in
        // the actual nearest-neighbour ordering at country scale, which is
        // all rstar needs.
        let dx = self.lon - p[0];
        let dy = self.lat - p[1];
        dx * dx + dy * dy
    }
}

pub struct Graph {
    pub graph: UnGraph<LatLon, EdgeData>,
    pub osm_to_idx: FxHashMap<i64, NodeIndex>,
    /// Per-NodeIndex component representative. Computed once at build time so
    /// every routing query is O(1) for "which component is this node in".
    pub components: Vec<u32>,
    /// Spatial index over node coordinates for fast nearest-neighbour queries.
    pub rtree: rstar::RTree<NodePoint>,
}

impl Graph {
    /// Build the auxiliary structures (union-find + R-tree) that the routing
    /// queries depend on. Called once at graph construction.
    fn build_aux(graph: &UnGraph<LatLon, EdgeData>) -> (Vec<u32>, rstar::RTree<NodePoint>) {
        let n = graph.node_count();
        let mut uf: UnionFind<usize> = UnionFind::new(n);
        for er in graph.edge_references() {
            uf.union(er.source().index(), er.target().index());
        }
        let components: Vec<u32> = (0..n).map(|i| uf.find(i) as u32).collect();
        let points: Vec<NodePoint> = graph
            .node_indices()
            .map(|ni| {
                let p = graph[ni];
                NodePoint {
                    idx: ni,
                    lat: p.lat,
                    lon: p.lon,
                }
            })
            .collect();
        let rtree = rstar::RTree::bulk_load(points);
        (components, rtree)
    }

    /// Closest graph node to `p`, via the R-tree. None if graph is empty.
    pub fn closest(&self, p: LatLon) -> Option<NodeIndex> {
        self.rtree
            .nearest_neighbor(&[p.lon, p.lat])
            .map(|np| np.idx)
    }

    /// Find `(start_node, end_node)` in the same connected component, as close
    /// as possible to the given lat/lon endpoints. We pull the K nearest
    /// candidates near each endpoint from the R-tree, then keep the cheapest
    /// (start, end) pair whose components match. K=30 covers OSM's typical
    /// fragmentation around lakes / private property without going O(N).
    pub fn closest_connected_pair(
        &self,
        start: LatLon,
        end: LatLon,
    ) -> Option<(NodeIndex, NodeIndex, f64, f64)> {
        if self.graph.node_count() == 0 {
            return None;
        }
        let k = 30;
        let s_cands: Vec<NodePoint> = self
            .rtree
            .nearest_neighbor_iter(&[start.lon, start.lat])
            .take(k)
            .copied()
            .collect();
        let e_cands: Vec<NodePoint> = self
            .rtree
            .nearest_neighbor_iter(&[end.lon, end.lat])
            .take(k)
            .copied()
            .collect();
        let mut best: Option<(f64, NodeIndex, NodeIndex, f64, f64)> = None;
        for s in &s_cands {
            for e in &e_cands {
                if self.components[s.idx.index()] != self.components[e.idx.index()] {
                    continue;
                }
                let sd = haversine(start, LatLon::new(s.lat, s.lon));
                let ed = haversine(end, LatLon::new(e.lat, e.lon));
                let total = sd + ed;
                if best.map_or(true, |(bt, ..)| total < bt) {
                    best = Some((total, s.idx, e.idx, sd, ed));
                }
            }
        }
        best.map(|(_, si, ei, sd, ed)| (si, ei, sd, ed))
    }
}

fn classify_surface(tags: &osmpbfreader::Tags, highway: &str) -> u8 {
    if let Some(s) = tags.get("surface").map(|s| s.as_str()) {
        return match s {
            "asphalt" | "paved" | "concrete" | "concrete:plates" | "concrete:lanes"
            | "paving_stones" | "chipseal" | "metal" => SURFACE_PAVED,
            "sett" | "cobblestone" | "unhewn_cobblestone" => SURFACE_PAVED,
            "gravel" | "fine_gravel" | "compacted" | "pebblestone" => SURFACE_GRAVEL,
            "unpaved" | "dirt" | "ground" | "earth" | "mud" | "grass" | "sand"
            | "woodchips" | "bark_chips" => SURFACE_UNPAVED,
            _ => SURFACE_OTHER,
        };
    }
    // No explicit surface tag: infer from highway type.
    match highway {
        "path" | "footway" | "bridleway" | "steps" => SURFACE_PATH,
        "track" => match tags.get("tracktype").map(|s| s.as_str()) {
            // grade1: solid (concrete/asphalt). grade2: gravel/compacted.
            // grade3: dirt with vegetation. grade4-5: rough.
            Some("grade1") => SURFACE_PAVED,
            Some("grade2") => SURFACE_GRAVEL,
            Some("grade3" | "grade4" | "grade5") => SURFACE_UNPAVED,
            _ => SURFACE_GRAVEL, // tracks default to gravel — usually right
        },
        "cycleway" | "service" | "residential" | "unclassified" | "tertiary"
        | "secondary" | "primary" | "living_street" | "pedestrian" => SURFACE_PAVED,
        _ => SURFACE_OTHER,
    }
}

fn classify(tags: &osmpbfreader::Tags, want: u8) -> (u8, u8) {
    let route = tags.get("route").map(|s| s.as_str());
    if route == Some("ferry") {
        return (
            (MODE_FOOT | MODE_BIKE | MODE_FERRY) & (want | MODE_FERRY),
            SURFACE_FERRY,
        );
    }
    let Some(hw) = tags.get("highway").map(|s| s.as_str()) else {
        return (0, SURFACE_OTHER);
    };
    let access = tags.get("access").map(|s| s.as_str());
    if matches!(access, Some("no") | Some("private")) {
        return (0, SURFACE_OTHER);
    }
    if matches!(hw, "motorway" | "motorway_link") {
        return (0, SURFACE_OTHER);
    }
    let foot = tags.get("foot").map(|s| s.as_str());
    let bicycle = tags.get("bicycle").map(|s| s.as_str());

    let foot_default = matches!(
        hw,
        "path"
            | "footway"
            | "track"
            | "residential"
            | "unclassified"
            | "tertiary"
            | "secondary"
            | "pedestrian"
            | "living_street"
            | "service"
            | "cycleway"
            | "steps"
            | "bridleway"
    );
    let bike_default = matches!(
        hw,
        "path"
            | "track"
            | "residential"
            | "unclassified"
            | "tertiary"
            | "secondary"
            | "primary"
            | "cycleway"
            | "living_street"
            | "service"
    );
    let mtb_default = matches!(
        hw,
        "path" | "track" | "footway" | "cycleway" | "bridleway"
    );

    let allow = |default: bool, override_tag: Option<&str>| -> bool {
        match override_tag {
            Some("yes") | Some("designated") | Some("permissive") | Some("destination") => true,
            Some("no") | Some("private") | Some("dismount") => false,
            _ => default,
        }
    };

    let mut m = 0u8;
    if (want & MODE_FOOT) != 0 && allow(foot_default, foot) {
        m |= MODE_FOOT;
    }
    if (want & MODE_BIKE) != 0 && allow(bike_default, bicycle) {
        m |= MODE_BIKE;
    }
    if (want & MODE_MTB) != 0 && allow(mtb_default, bicycle) {
        m |= MODE_MTB;
    }
    let surface = classify_surface(tags, hw);
    (m, surface)
}

/// Build a routing graph from a PBF, bounded by a lat/lon bbox.
/// `keep_edge(p0, p1)` is the final edge predicate; return `true` to keep,
/// `false` to drop. Use this to restrict to a corridor in plan mode, or
/// `|_, _| true` in explore mode.
pub fn build<F>(
    path: &Path,
    bbox: BBox,
    want_modes: u8,
    keep_edge: F,
) -> Result<Graph>
where
    F: Fn(LatLon, LatLon) -> bool,
{
    let BBox {
        lat_min,
        lat_max,
        lon_min,
        lon_max,
    } = bbox;
    eprintln!(
        "Reading PBF (pass 1: ways) — bbox [{:.3},{:.3}]×[{:.3},{:.3}]",
        lat_min, lat_max, lon_min, lon_max
    );

    // Pass 1: find ways with passing tags, collect their node IDs, mode mask, surface.
    let mut keep_ways: Vec<(i64, Vec<i64>, u8, u8)> = Vec::new();
    let mut needed_nodes: FxHashSet<i64> = FxHashSet::default();
    {
        let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
        let mut pbf = OsmPbfReader::new(f);
        let pb = ProgressBar::new_spinner();
        pb.set_style(
            ProgressStyle::with_template("{spinner} pass1: {msg} ({pos} ways scanned)")
                .unwrap(),
        );
        pb.enable_steady_tick(std::time::Duration::from_millis(120));
        let mut scanned = 0u64;
        for obj in pbf.iter() {
            let obj = obj?;
            if let OsmObj::Way(w) = obj {
                scanned += 1;
                if scanned % 50_000 == 0 {
                    pb.set_position(scanned);
                    pb.set_message(format!("kept {}", keep_ways.len()));
                }
                let (m, s) = classify(&w.tags, want_modes);
                if m == 0 {
                    continue;
                }
                let nodes: Vec<i64> = w.nodes.iter().map(|n| n.0).collect();
                for n in &nodes {
                    needed_nodes.insert(*n);
                }
                keep_ways.push((w.id.0, nodes, m, s));
            }
        }
        pb.finish_with_message(format!(
            "kept {} ways referencing {} nodes",
            keep_ways.len(),
            needed_nodes.len()
        ));
    }

    eprintln!("Reading PBF (pass 2: nodes)");
    // Pass 2: read needed node coords.
    let mut node_pos: FxHashMap<i64, LatLon> =
        FxHashMap::with_capacity_and_hasher(needed_nodes.len(), Default::default());
    {
        let f = File::open(path)?;
        let mut pbf = OsmPbfReader::new(f);
        let pb = ProgressBar::new(needed_nodes.len() as u64);
        pb.set_style(
            ProgressStyle::with_template("pass2: {bar:40} {pos}/{len} nodes resolved")
                .unwrap(),
        );
        for obj in pbf.iter() {
            let obj = obj?;
            if let OsmObj::Node(n) = obj {
                if needed_nodes.contains(&n.id.0) {
                    let lat = n.decimicro_lat as f64 * 1e-7;
                    let lon = n.decimicro_lon as f64 * 1e-7;
                    if lat >= lat_min && lat <= lat_max && lon >= lon_min && lon <= lon_max {
                        node_pos.insert(n.id.0, LatLon::new(lat, lon));
                        if node_pos.len() % 100_000 == 0 {
                            pb.set_position(node_pos.len() as u64);
                        }
                    }
                }
            }
        }
        pb.finish_with_message(format!("resolved {} nodes in bbox", node_pos.len()));
    }

    eprintln!("Building graph (final edge filter)");
    let mut graph: UnGraph<LatLon, EdgeData> =
        UnGraph::with_capacity(node_pos.len(), node_pos.len() * 2);
    let mut osm_to_idx: FxHashMap<i64, NodeIndex> =
        FxHashMap::with_capacity_and_hasher(node_pos.len(), Default::default());

    let get_or_insert = |graph: &mut UnGraph<LatLon, EdgeData>,
                         osm_to_idx: &mut FxHashMap<i64, NodeIndex>,
                         id: i64,
                         p: LatLon|
     -> NodeIndex {
        if let Some(idx) = osm_to_idx.get(&id) {
            return *idx;
        }
        let idx = graph.add_node(p);
        osm_to_idx.insert(id, idx);
        idx
    };

    let mut edges_added = 0usize;
    for (way_id, nodes, modes, surface) in &keep_ways {
        for w in nodes.windows(2) {
            let (Some(p0), Some(p1)) = (node_pos.get(&w[0]).copied(), node_pos.get(&w[1]).copied())
            else {
                continue;
            };
            if !keep_edge(p0, p1) {
                continue;
            }
            let length = haversine(p0, p1);
            if length == 0.0 {
                continue;
            }
            let i0 = get_or_insert(&mut graph, &mut osm_to_idx, w[0], p0);
            let i1 = get_or_insert(&mut graph, &mut osm_to_idx, w[1], p1);
            graph.add_edge(
                i0,
                i1,
                EdgeData {
                    length_m: length,
                    modes: *modes,
                    surface: *surface,
                    way_id: *way_id,
                },
            );
            edges_added += 1;
        }
    }

    eprintln!(
        "Graph built: {} nodes, {} edges",
        graph.node_count(),
        edges_added
    );

    eprintln!("Computing connected components + spatial index…");
    let (components, rtree) = Graph::build_aux(&graph);

    Ok(Graph {
        graph,
        osm_to_idx,
        components,
        rtree,
    })
}
