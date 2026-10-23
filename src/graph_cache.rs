//! On-disk cache of the routing graph `serve` builds from a PBF.
//!
//! Parsing the Switzerland extract and building its graph takes 1–2 min;
//! reading the cache back takes seconds. Only nodes and edges are stored:
//! components and the R-tree are recomputed on load, and DEM elevations are
//! stamped afterwards as on a fresh build. Nodes and edges are re-added in
//! index order, so the loaded graph — adjacency order included — and every
//! route over it are identical to a fresh build.
//!
//! A cache is used only when its key matches: the PBF's size and mtime, the
//! mode mask, and a hash of the graph-building source (`osm.rs`,
//! `geodesy.rs`), so a change to the OSM classification rules can never
//! silently reuse a stale graph. Anything unexpected falls back to a build.

use crate::geodesy::LatLon;
use crate::osm::{self, BBox, EdgeData, Graph};
use anyhow::{anyhow, Context, Result};
use petgraph::graph::{NodeIndex, UnGraph};
use petgraph::visit::EdgeRef;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, ErrorKind, Read, Write};
use std::path::{Path, PathBuf};
use std::time::UNIX_EPOCH;

const MAGIC: &[u8; 8] = b"CROWFLYG";
const FORMAT_VERSION: u32 = 1;
/// Serialized edge: two u32 endpoints, f64 length, modes, surface, bike_attrs.
const EDGE_BYTES: usize = 4 + 4 + 8 + 3;

/// Build the graph of every way in `pbf` usable by `modes`, or load it from
/// the cache in `cache_dir` when one with a matching key exists (writing a
/// fresh cache after a build). `cache_dir: None` always builds.
pub fn build_or_load(pbf: &Path, modes: u8, cache_dir: Option<&Path>) -> Result<Graph> {
    let everything = BBox {
        lat_min: -85.0,
        lat_max: 85.0,
        lon_min: -180.0,
        lon_max: 180.0,
    };
    let Some(cache_dir) = cache_dir else {
        return osm::build(pbf, everything, modes, |_, _| true);
    };
    let path = cache_path(cache_dir, pbf);
    let key = cache_key(pbf, modes)?;
    let started = std::time::Instant::now();
    match load(&path, key) {
        Ok(Some(g)) => {
            eprintln!(
                "Loaded graph cache {} in {:.1}s",
                path.display(),
                started.elapsed().as_secs_f64()
            );
            return Ok(g);
        }
        Ok(None) => {}
        Err(e) => eprintln!(
            "  (ignoring unreadable graph cache {}: {e})",
            path.display()
        ),
    }
    let g = osm::build(pbf, everything, modes, |_, _| true)?;
    match save(&g, &path, key) {
        Ok(()) => eprintln!("Saved graph cache {}", path.display()),
        Err(e) => eprintln!("  (could not write graph cache {}: {e})", path.display()),
    }
    Ok(g)
}

/// `<cache_dir>/<pbf file name>.graph`.
pub fn cache_path(cache_dir: &Path, pbf: &Path) -> PathBuf {
    let name = pbf
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "graph".into());
    cache_dir.join(format!("{name}.graph"))
}

/// Identity of the inputs a cached graph was built from.
pub fn cache_key(pbf: &Path, modes: u8) -> Result<u64> {
    let meta = fs::metadata(pbf).with_context(|| format!("reading {}", pbf.display()))?;
    let mtime_ns = meta
        .modified()
        .ok()
        .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
        .map_or(0, |d| d.as_nanos() as u64);
    let mut h = FNV_OFFSET;
    h = fnv1a(h, &meta.len().to_le_bytes());
    h = fnv1a(h, &mtime_ns.to_le_bytes());
    h = fnv1a(h, &[modes]);
    h = fnv1a(h, include_str!("osm.rs").as_bytes());
    h = fnv1a(h, include_str!("geodesy.rs").as_bytes());
    Ok(h)
}

const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;

fn fnv1a(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Write `g`'s nodes and edges under `key`, via a temp file + rename so a
/// crash mid-write never leaves a truncated cache behind.
pub fn save(g: &Graph, path: &Path, key: u64) -> Result<()> {
    if let Some(dir) = path.parent() {
        fs::create_dir_all(dir)?;
    }
    let tmp = path.with_extension("graph.part");
    {
        let mut w = BufWriter::with_capacity(1 << 20, File::create(&tmp)?);
        w.write_all(MAGIC)?;
        w.write_all(&FORMAT_VERSION.to_le_bytes())?;
        w.write_all(&key.to_le_bytes())?;
        w.write_all(&(g.graph.node_count() as u64).to_le_bytes())?;
        w.write_all(&(g.graph.edge_count() as u64).to_le_bytes())?;
        for p in g.graph.node_weights() {
            w.write_all(&p.lat.to_le_bytes())?;
            w.write_all(&p.lon.to_le_bytes())?;
        }
        for e in g.graph.edge_references() {
            let d = e.weight();
            w.write_all(&(e.source().index() as u32).to_le_bytes())?;
            w.write_all(&(e.target().index() as u32).to_le_bytes())?;
            w.write_all(&d.length_m.to_le_bytes())?;
            w.write_all(&[d.modes, d.surface, d.bike_attrs])?;
        }
        w.flush()?;
    }
    fs::rename(&tmp, path)?;
    Ok(())
}

/// Read a cache written by [`save`]. `Ok(None)` when it is missing or was
/// written for a different key/format; `Err` when it is corrupt.
pub fn load(path: &Path, key: u64) -> Result<Option<Graph>> {
    let file = match File::open(path) {
        Ok(f) => f,
        Err(e) if e.kind() == ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e.into()),
    };
    let mut r = BufReader::with_capacity(1 << 20, file);
    let mut magic = [0u8; 8];
    r.read_exact(&mut magic)?;
    if &magic != MAGIC || read_u32(&mut r)? != FORMAT_VERSION || read_u64(&mut r)? != key {
        return Ok(None);
    }
    let n_nodes = read_u64(&mut r)? as usize;
    let n_edges = read_u64(&mut r)? as usize;
    let mut graph: UnGraph<LatLon, EdgeData> = UnGraph::with_capacity(n_nodes, n_edges);
    let mut nb = [0u8; 16];
    for _ in 0..n_nodes {
        r.read_exact(&mut nb)?;
        graph.add_node(LatLon::new(f64_at(&nb, 0), f64_at(&nb, 8)));
    }
    let mut eb = [0u8; EDGE_BYTES];
    for _ in 0..n_edges {
        r.read_exact(&mut eb)?;
        let a = u32::from_le_bytes(eb[0..4].try_into().unwrap()) as usize;
        let b = u32::from_le_bytes(eb[4..8].try_into().unwrap()) as usize;
        if a >= n_nodes || b >= n_nodes {
            return Err(anyhow!("edge endpoint out of range"));
        }
        graph.add_edge(
            NodeIndex::new(a),
            NodeIndex::new(b),
            EdgeData {
                length_m: f64_at(&eb, 8),
                modes: eb[16],
                surface: eb[17],
                bike_attrs: eb[18],
            },
        );
    }
    Ok(Some(Graph::from_graph(graph)))
}

fn f64_at(buf: &[u8], at: usize) -> f64 {
    f64::from_le_bytes(buf[at..at + 8].try_into().unwrap())
}

fn read_u32(r: &mut impl Read) -> Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

fn read_u64(r: &mut impl Read) -> Result<u64> {
    let mut b = [0u8; 8];
    r.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::{BCLASS_CYCLEWAY, BCLASS_PATH, MODE_BIKE, MODE_FOOT, SURFACE_PAVED};

    fn sample_graph() -> Graph {
        let mut g: UnGraph<LatLon, EdgeData> = UnGraph::new_undirected();
        let a = g.add_node(LatLon::new(47.1, 9.5));
        let b = g.add_node(LatLon::new(47.2, 9.6));
        let c = g.add_node(LatLon::new(47.15, 9.55));
        let edge = |len, modes, attrs| EdgeData {
            length_m: len,
            modes,
            surface: SURFACE_PAVED,
            bike_attrs: attrs,
        };
        g.add_edge(a, b, edge(1234.5, MODE_FOOT | MODE_BIKE, BCLASS_CYCLEWAY));
        g.add_edge(b, c, edge(42.0, MODE_FOOT, BCLASS_PATH));
        g.add_edge(a, b, edge(1300.0, MODE_BIKE, BCLASS_CYCLEWAY)); // parallel edge
        Graph::from_graph(g)
    }

    fn temp_path(name: &str) -> PathBuf {
        std::env::temp_dir().join(format!("crowfly-test-{}-{name}.graph", std::process::id()))
    }

    #[test]
    fn roundtrip_preserves_nodes_edges_and_order() {
        let g = sample_graph();
        let path = temp_path("roundtrip");
        save(&g, &path, 7).unwrap();
        let back = load(&path, 7).unwrap().expect("cache hit");
        let _ = fs::remove_file(&path);

        let nodes = |g: &Graph| g.graph.node_weights().copied().collect::<Vec<_>>();
        assert_eq!(nodes(&back), nodes(&g));
        let edges = |g: &Graph| {
            g.graph
                .edge_references()
                .map(|e| {
                    let d = e.weight();
                    (
                        e.source(),
                        e.target(),
                        d.length_m,
                        d.modes,
                        d.surface,
                        d.bike_attrs,
                    )
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(edges(&back), edges(&g));
        // Adjacency order drives A* tie-breaking, so it must match too.
        let adj = |g: &Graph| {
            g.graph
                .node_indices()
                .map(|n| g.graph.edges(n).map(|e| e.id()).collect::<Vec<_>>())
                .collect::<Vec<_>>()
        };
        assert_eq!(adj(&back), adj(&g));
        assert_eq!(back.components.len(), 3);
    }

    #[test]
    fn stale_or_missing_cache_is_a_miss() {
        let path = temp_path("stale");
        assert!(load(&path, 1).unwrap().is_none(), "missing file");
        save(&sample_graph(), &path, 1).unwrap();
        assert!(load(&path, 2).unwrap().is_none(), "different key");
        let _ = fs::remove_file(&path);
    }

    #[test]
    fn truncated_cache_is_an_error() {
        let path = temp_path("truncated");
        save(&sample_graph(), &path, 1).unwrap();
        let bytes = fs::read(&path).unwrap();
        fs::write(&path, &bytes[..bytes.len() - 5]).unwrap();
        assert!(load(&path, 1).is_err());
        let _ = fs::remove_file(&path);
    }

    #[test]
    fn key_tracks_pbf_and_modes() {
        let path = temp_path("pbf");
        fs::write(&path, b"not really a pbf").unwrap();
        let k1 = cache_key(&path, MODE_FOOT).unwrap();
        assert_eq!(k1, cache_key(&path, MODE_FOOT).unwrap());
        assert_ne!(k1, cache_key(&path, MODE_FOOT | MODE_BIKE).unwrap());
        fs::write(&path, b"a different, longer pbf").unwrap();
        assert_ne!(k1, cache_key(&path, MODE_FOOT).unwrap());
        let _ = fs::remove_file(&path);
    }
}
