//! Read an OSM PBF and build a corridor-bounded routing graph.

use crate::geodesy::{haversine, LatLon};
use anyhow::{Context, Result};
use geo::{Contains, Coord, LineString, Polygon};
use indicatif::{ProgressBar, ProgressStyle};
use osmpbfreader::{OsmObj, OsmPbfReader};
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::unionfind::UnionFind;
use petgraph::visit::EdgeRef;
use rstar::{RTree, RTreeObject, AABB};
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

/// Coarse bike-routing class derived from `highway=*` plus bicycle access tags.
/// Stored in the low 4 bits of `EdgeData::bike_attrs`. The router uses these
/// to apply per-class cost multipliers (cycleway/residential favoured,
/// primary/secondary penalised), so suggestions reflect what cyclists actually
/// pick — not just the cartographically shortest line.
pub const BCLASS_CYCLEWAY: u8 = 0;
pub const BCLASS_RESIDENTIAL: u8 = 1; // residential, living_street
pub const BCLASS_TERTIARY: u8 = 2; // tertiary, unclassified
pub const BCLASS_SERVICE: u8 = 3; // service
pub const BCLASS_SECONDARY: u8 = 4;
pub const BCLASS_PRIMARY: u8 = 5;
pub const BCLASS_TRACK: u8 = 6; // track, bridleway
pub const BCLASS_PATH: u8 = 7; // path, footway, pedestrian, steps
pub const BCLASS_FERRY: u8 = 8;
pub const BCLASS_OTHER: u8 = 9;
pub const BCLASS_MASK: u8 = 0x0F;
/// `bicycle=designated` set on the way — strong cyclist preference, regardless
/// of highway class. Stored in bit 4 of `bike_attrs`.
pub const BATTR_DESIGNATED: u8 = 1 << 4;
/// Way is part of a marked cycle network (`lcn`/`rcn`/`ncn=yes` on the way
/// itself). Stored in bit 5. Relation membership is *not* parsed — only
/// way-level tags, which is what most local OSM mappers set on signed routes.
pub const BATTR_CYCLE_NETWORK: u8 = 1 << 5;

#[derive(Copy, Clone, Debug)]
pub struct EdgeData {
    pub length_m: f64,
    pub modes: u8,
    pub surface: u8,
    /// Packed bike-routing attributes: low 4 bits = `BCLASS_*`, bit 4 =
    /// `BATTR_DESIGNATED`, bit 5 = `BATTR_CYCLE_NETWORK`. See module-level
    /// constants. Zero for non-bike-routable edges (foot-only ferries, etc.)
    /// and harmless because `route.rs` only consults this byte when bike or
    /// mtb is in the active mode mask.
    pub bike_attrs: u8,
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
    /// Per-NodeIndex elevation in metres, populated by `stamp_elevations`.
    /// `None` until a DEM has been loaded; `f32::NAN` for nodes outside the
    /// loaded tile coverage. The router uses these to penalise descending on
    /// non-rideable surfaces when bike is in the mode set.
    pub elevations_m: Option<Vec<f32>>,
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

    /// Stamp every graph node with its elevation (metres) from the supplied
    /// DEM. Nodes that fall outside the DEM's tile coverage get `f32::NAN`.
    /// Logs a coverage summary including which integer-degree tiles are
    /// missing so the user knows what to download next.
    pub fn stamp_elevations(&mut self, dem: &crate::dem::Dem) {
        let n = self.graph.node_count();
        let mut elev: Vec<f32> = Vec::with_capacity(n);
        let mut hits = 0usize;
        let mut missing_tiles: FxHashSet<(i32, i32)> = FxHashSet::default();
        for ni in self.graph.node_indices() {
            let p = self.graph[ni];
            match dem.elevation_at(p) {
                Some(z) => {
                    elev.push(z as f32);
                    hits += 1;
                }
                None => {
                    elev.push(f32::NAN);
                    missing_tiles.insert((p.lat.floor() as i32, p.lon.floor() as i32));
                }
            }
        }
        eprintln!(
            "DEM coverage: {}/{} nodes have elevation ({:.1}%)",
            hits,
            n,
            100.0 * hits as f64 / (n.max(1)) as f64
        );
        if !missing_tiles.is_empty() {
            let mut list: Vec<String> = missing_tiles
                .iter()
                .map(|(lat, lon)| format!("{}{:02}{}{:03}",
                    if *lat >= 0 { 'N' } else { 'S' }, lat.abs(),
                    if *lon >= 0 { 'E' } else { 'W' }, lon.abs()))
                .collect();
            list.sort();
            eprintln!("  (missing tiles: {})", list.join(", "));
        }
        self.elevations_m = Some(elev);
    }

    /// Low-pass filter `elevations_m` along the road graph: each node becomes
    /// the average of itself plus its routed neighbours. DEM noise (~1–2 m at
    /// SRTM-30 m resolution) on edges only 5–20 m long otherwise registers as
    /// 15–20 % "phantom grades" and skews the cost function's ascent term —
    /// the router would obediently treat each spurious spike as a wall to
    /// detour around. One or two passes is enough to wash out that noise
    /// without flattening genuine climbs (the averaging window is one edge
    /// wide so a 2 km hill still reads as a hill).
    pub fn smooth_elevations(&mut self, iterations: usize) {
        let Some(elev) = self.elevations_m.as_mut() else {
            return;
        };
        if iterations == 0 || elev.is_empty() {
            return;
        }
        let n = elev.len();
        let mut next: Vec<f32> = vec![f32::NAN; n];
        for _ in 0..iterations {
            for ni in self.graph.node_indices() {
                let i = ni.index();
                let mut sum = 0.0f64;
                let mut count = 0u32;
                let zi = elev[i];
                if zi.is_finite() {
                    sum += zi as f64;
                    count += 1;
                }
                for nbr in self.graph.neighbors(ni) {
                    let z = elev[nbr.index()];
                    if z.is_finite() {
                        sum += z as f64;
                        count += 1;
                    }
                }
                next[i] = if count > 0 {
                    (sum / count as f64) as f32
                } else {
                    f32::NAN
                };
            }
            std::mem::swap(elev, &mut next);
        }
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

fn classify(tags: &osmpbfreader::Tags, want: u8) -> (u8, u8, u8) {
    let route = tags.get("route").map(|s| s.as_str());
    if route == Some("ferry") {
        return (
            (MODE_FOOT | MODE_BIKE | MODE_FERRY) & (want | MODE_FERRY),
            SURFACE_FERRY,
            BCLASS_FERRY,
        );
    }
    let Some(hw) = tags.get("highway").map(|s| s.as_str()) else {
        return (0, SURFACE_OTHER, BCLASS_OTHER);
    };
    let access = tags.get("access").map(|s| s.as_str());
    if matches!(access, Some("no") | Some("private")) {
        return (0, SURFACE_OTHER, BCLASS_OTHER);
    }
    if matches!(hw, "motorway" | "motorway_link") {
        return (0, SURFACE_OTHER, BCLASS_OTHER);
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

    let class = match hw {
        "cycleway" => BCLASS_CYCLEWAY,
        "residential" | "living_street" => BCLASS_RESIDENTIAL,
        "tertiary" | "tertiary_link" | "unclassified" => BCLASS_TERTIARY,
        "service" => BCLASS_SERVICE,
        "secondary" | "secondary_link" => BCLASS_SECONDARY,
        "primary" | "primary_link" | "trunk" | "trunk_link" => BCLASS_PRIMARY,
        "track" | "bridleway" => BCLASS_TRACK,
        "path" | "footway" | "pedestrian" | "steps" => BCLASS_PATH,
        _ => BCLASS_OTHER,
    };
    let mut attrs = class;
    if matches!(bicycle, Some("designated")) {
        attrs |= BATTR_DESIGNATED;
    }
    let net = |k: &str| matches!(tags.get(k).map(|s| s.as_str()), Some("yes"));
    if net("lcn") || net("rcn") || net("ncn") {
        attrs |= BATTR_CYCLE_NETWORK;
    }
    (m, surface, attrs)
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
    // Also collect `natural=glacier` closed ways so we can skip edges that cross
    // them — glaciers aren't passable terrain even if a path is tagged through them.
    let mut keep_ways: Vec<(i64, Vec<i64>, u8, u8, u8)> = Vec::new();
    let mut glacier_ways: Vec<Vec<i64>> = Vec::new();
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
                if w.tags.get("natural").map(|s| s.as_str()) == Some("glacier") {
                    let nodes: Vec<i64> = w.nodes.iter().map(|n| n.0).collect();
                    if nodes.len() >= 3 {
                        for n in &nodes {
                            needed_nodes.insert(*n);
                        }
                        glacier_ways.push(nodes);
                    }
                    continue;
                }
                let (m, s, ba) = classify(&w.tags, want_modes);
                if m == 0 {
                    continue;
                }
                let nodes: Vec<i64> = w.nodes.iter().map(|n| n.0).collect();
                for n in &nodes {
                    needed_nodes.insert(*n);
                }
                keep_ways.push((w.id.0, nodes, m, s, ba));
            }
        }
        pb.finish_with_message(format!(
            "kept {} ways referencing {} nodes (+{} glacier ways)",
            keep_ways.len(),
            needed_nodes.len(),
            glacier_ways.len(),
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

    // Build glacier polygons + spatial index from the closed ways collected in
    // pass 1. Edges whose midpoint falls inside any of these polygons are
    // dropped from the graph, so the router can't suggest a path that goes
    // over a glacier (even if OSM has a track tagged across it).
    let glacier_index = build_glacier_index(&glacier_ways, &node_pos);
    if !glacier_index.polys.is_empty() {
        eprintln!(
            "Glacier exclusion: {} polygons indexed",
            glacier_index.polys.len()
        );
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
    let mut edges_skipped_glacier = 0usize;
    for (way_id, nodes, modes, surface, bike_attrs) in &keep_ways {
        for w in nodes.windows(2) {
            let (Some(p0), Some(p1)) = (node_pos.get(&w[0]).copied(), node_pos.get(&w[1]).copied())
            else {
                continue;
            };
            if !keep_edge(p0, p1) {
                continue;
            }
            let mid = LatLon::new((p0.lat + p1.lat) * 0.5, (p0.lon + p1.lon) * 0.5);
            if glacier_index.contains(mid) {
                edges_skipped_glacier += 1;
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
                    bike_attrs: *bike_attrs,
                    way_id: *way_id,
                },
            );
            edges_added += 1;
        }
    }

    eprintln!(
        "Graph built: {} nodes, {} edges ({} skipped over glaciers)",
        graph.node_count(),
        edges_added,
        edges_skipped_glacier,
    );

    eprintln!("Computing connected components + spatial index…");
    let (components, rtree) = Graph::build_aux(&graph);

    Ok(Graph {
        graph,
        osm_to_idx,
        components,
        rtree,
        elevations_m: None,
    })
}

/// Spatial index over `natural=glacier` polygons. AABB pre-filter via rstar,
/// precise containment via `geo::Contains` for the small candidate set.
struct GlacierIndex {
    polys: Vec<Polygon<f64>>,
    rtree: RTree<GlacierBox>,
}

#[derive(Clone, Copy)]
struct GlacierBox {
    min: [f64; 2],
    max: [f64; 2],
    idx: usize,
}

impl RTreeObject for GlacierBox {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.min, self.max)
    }
}

impl GlacierIndex {
    fn empty() -> Self {
        Self {
            polys: Vec::new(),
            rtree: RTree::new(),
        }
    }
    fn contains(&self, p: LatLon) -> bool {
        if self.polys.is_empty() {
            return false;
        }
        let pt = [p.lon, p.lat];
        let env = AABB::from_point(pt);
        for gb in self.rtree.locate_in_envelope_intersecting(&env) {
            let coord = Coord { x: p.lon, y: p.lat };
            if self.polys[gb.idx].contains(&coord) {
                return true;
            }
        }
        false
    }
}

fn build_glacier_index(
    glacier_ways: &[Vec<i64>],
    node_pos: &FxHashMap<i64, LatLon>,
) -> GlacierIndex {
    if glacier_ways.is_empty() {
        return GlacierIndex::empty();
    }
    let mut polys: Vec<Polygon<f64>> = Vec::new();
    let mut boxes: Vec<GlacierBox> = Vec::new();
    for nodes in glacier_ways {
        let mut coords: Vec<Coord<f64>> = Vec::with_capacity(nodes.len());
        for id in nodes {
            if let Some(p) = node_pos.get(id) {
                coords.push(Coord { x: p.lon, y: p.lat });
            }
        }
        if coords.len() < 3 {
            continue;
        }
        // Close the ring if the way isn't already closed (most are, but the
        // bbox-clipping in pass 2 can drop trailing nodes).
        if coords.first() != coords.last() {
            coords.push(coords[0]);
        }
        let mut lon_min = f64::INFINITY;
        let mut lon_max = f64::NEG_INFINITY;
        let mut lat_min = f64::INFINITY;
        let mut lat_max = f64::NEG_INFINITY;
        for c in &coords {
            if c.x < lon_min { lon_min = c.x; }
            if c.x > lon_max { lon_max = c.x; }
            if c.y < lat_min { lat_min = c.y; }
            if c.y > lat_max { lat_max = c.y; }
        }
        let idx = polys.len();
        polys.push(Polygon::new(LineString::from(coords), vec![]));
        boxes.push(GlacierBox {
            min: [lon_min, lat_min],
            max: [lon_max, lat_max],
            idx,
        });
    }
    let rtree = RTree::bulk_load(boxes);
    GlacierIndex { polys, rtree }
}

#[cfg(test)]
mod tests {
    use super::*;
    use petgraph::graph::UnGraph;

    fn tiny_chain(n: usize) -> Graph {
        // n linearly-connected nodes spaced 0.001° apart along the equator.
        let mut g: UnGraph<LatLon, EdgeData> = UnGraph::new_undirected();
        let mut ids = Vec::with_capacity(n);
        for i in 0..n {
            ids.push(g.add_node(LatLon::new(0.0, i as f64 * 0.001)));
        }
        for w in ids.windows(2) {
            let length = crate::geodesy::haversine(g[w[0]], g[w[1]]);
            g.add_edge(
                w[0],
                w[1],
                EdgeData {
                    length_m: length,
                    modes: MODE_FOOT,
                    surface: SURFACE_PAVED,
                    bike_attrs: BCLASS_RESIDENTIAL,
                    way_id: 1,
                },
            );
        }
        let (components, rtree) = Graph::build_aux(&g);
        Graph {
            graph: g,
            osm_to_idx: rustc_hash::FxHashMap::default(),
            components,
            rtree,
            elevations_m: None,
        }
    }

    #[test]
    fn smooth_elevations_averages_with_neighbours() {
        // Three-node chain: A — B — C. B carries a 100 m elevation spike
        // that A and C do not. One smoothing iteration should pull B's
        // value down to the mean of (A_old, B_old, C_old) = (100+200+100)/3.
        let mut g = tiny_chain(3);
        g.elevations_m = Some(vec![100.0, 200.0, 100.0]);
        g.smooth_elevations(1);
        let elev = g.elevations_m.as_ref().unwrap();
        assert!((elev[1] - 133.333).abs() < 0.5,
            "B should be averaged toward neighbours, got {}", elev[1]);
        // Endpoints have a single neighbour, so they average to (own + nbr) / 2.
        assert!((elev[0] - 150.0).abs() < 0.5, "A averaged with B → 150, got {}", elev[0]);
        assert!((elev[2] - 150.0).abs() < 0.5, "C averaged with B → 150, got {}", elev[2]);
    }

    #[test]
    fn smooth_elevations_is_noop_on_flat_terrain() {
        // If every node is already at the same elevation, smoothing must
        // not introduce noise.
        let mut g = tiny_chain(5);
        g.elevations_m = Some(vec![500.0, 500.0, 500.0, 500.0, 500.0]);
        g.smooth_elevations(3);
        for z in g.elevations_m.as_ref().unwrap() {
            assert!((z - 500.0).abs() < 1e-3, "flat terrain should stay flat, got {}", z);
        }
    }

    #[test]
    fn smooth_elevations_preserves_nan_when_isolated() {
        // No DEM coverage anywhere → smoothing is a no-op (and must not panic
        // or replace NaN with 0).
        let mut g = tiny_chain(3);
        g.elevations_m = Some(vec![f32::NAN; 3]);
        g.smooth_elevations(2);
        for z in g.elevations_m.as_ref().unwrap() {
            assert!(z.is_nan(), "without any finite neighbour we should preserve NaN");
        }
    }

    #[test]
    fn classify_marks_cycleway() {
        let mut tags = osmpbfreader::Tags::new();
        tags.insert("highway".into(), "cycleway".into());
        let (modes, surface, attrs) = classify(&tags, MODE_FOOT | MODE_BIKE | MODE_MTB);
        assert!(modes & MODE_BIKE != 0, "cycleway must be bike-routable");
        assert_eq!(surface, SURFACE_PAVED, "cycleway defaults to paved");
        assert_eq!(attrs & BCLASS_MASK, BCLASS_CYCLEWAY);
    }

    #[test]
    fn classify_marks_primary_as_primary_class() {
        let mut tags = osmpbfreader::Tags::new();
        tags.insert("highway".into(), "primary".into());
        let (modes, _surface, attrs) = classify(&tags, MODE_BIKE);
        assert!(modes & MODE_BIKE != 0, "primary is bike-default-allowed");
        assert_eq!(attrs & BCLASS_MASK, BCLASS_PRIMARY);
    }

    #[test]
    fn classify_records_designated_and_network_flags() {
        let mut tags = osmpbfreader::Tags::new();
        tags.insert("highway".into(), "tertiary".into());
        tags.insert("bicycle".into(), "designated".into());
        tags.insert("lcn".into(), "yes".into());
        let (_modes, _surface, attrs) = classify(&tags, MODE_BIKE);
        assert!(attrs & BATTR_DESIGNATED != 0, "bicycle=designated must set flag");
        assert!(attrs & BATTR_CYCLE_NETWORK != 0, "lcn=yes must set network flag");
    }

    #[test]
    fn classify_rejects_motorway() {
        let mut tags = osmpbfreader::Tags::new();
        tags.insert("highway".into(), "motorway".into());
        let (modes, _, _) = classify(&tags, MODE_FOOT | MODE_BIKE | MODE_MTB);
        assert_eq!(modes, 0, "motorway must be unroutable for foot/bike/mtb");
    }
}
