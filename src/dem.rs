//! Digital elevation model loaded from SRTM .hgt tiles.
//!
//! `.hgt` is a flat, headerless 16-bit big-endian signed-int grid covering
//! exactly one 1°×1° square. 1201 samples per side = SRTM3 (3-arc-second,
//! ~90 m); 3601 = SRTM1 (1-arc-second, ~30 m). The first sample is the
//! north-west corner; rows run north→south, columns west→east. Voids use
//! `i16::MIN`.
//!
//! Filenames encode the SOUTH-WEST corner: `N46E007.hgt` covers
//! lat ∈ [46, 47), lon ∈ [7, 8). We read every `.hgt` under a directory
//! once at startup, stamp the corresponding graph node elevations, and the
//! router uses those to penalise descending edges on non-rideable surfaces.

use crate::geodesy::LatLon;
use anyhow::{anyhow, Context, Result};
use indicatif::{ProgressBar, ProgressStyle};
use rustc_hash::FxHashMap;
use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

const VOID: i16 = i16::MIN;

#[derive(Debug, Clone)]
pub struct Tile {
    pub lat0: i32,
    pub lon0: i32,
    pub side: usize,
    /// Row-major, north→south then west→east. Length = side*side.
    pub data: Vec<i16>,
}

impl Tile {
    fn elevation_at(&self, p: LatLon) -> Option<f64> {
        let lat0 = self.lat0 as f64;
        let lon0 = self.lon0 as f64;
        if p.lat < lat0 || p.lat >= lat0 + 1.0 || p.lon < lon0 || p.lon >= lon0 + 1.0 {
            return None;
        }
        let n = self.side as f64;
        let lat_frac = p.lat - lat0;
        let lon_frac = p.lon - lon0;
        // Row 0 is the north edge.
        let row_f = (1.0 - lat_frac) * (n - 1.0);
        let col_f = lon_frac * (n - 1.0);
        let r0 = row_f.floor() as usize;
        let c0 = col_f.floor() as usize;
        let r1 = (r0 + 1).min(self.side - 1);
        let c1 = (c0 + 1).min(self.side - 1);
        let dr = row_f - r0 as f64;
        let dc = col_f - c0 as f64;
        let v = |r: usize, c: usize| -> Option<f64> {
            let z = self.data[r * self.side + c];
            if z == VOID { None } else { Some(z as f64) }
        };
        // Bilinear interpolation that drops voided neighbours.
        let mut sum = 0.0;
        let mut w = 0.0;
        for &(rr, cc, ww) in &[
            (r0, c0, (1.0 - dr) * (1.0 - dc)),
            (r0, c1, (1.0 - dr) * dc),
            (r1, c0, dr * (1.0 - dc)),
            (r1, c1, dr * dc),
        ] {
            if let Some(z) = v(rr, cc) {
                sum += z * ww;
                w += ww;
            }
        }
        if w > 0.0 { Some(sum / w) } else { None }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Dem {
    pub tiles: FxHashMap<(i32, i32), Tile>,
}

impl Dem {
    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }

    /// Read every `*.hgt` file under `dir` (recursively). Filenames must
    /// follow the SRTM naming convention (`NxxEyyy.hgt`); anything else is
    /// skipped with a warning.
    pub fn load_dir(dir: &Path) -> Result<Self> {
        if !dir.exists() {
            return Err(anyhow!("DEM directory does not exist: {}", dir.display()));
        }
        let mut dem = Self::default();
        let mut hgt_paths: Vec<PathBuf> = Vec::new();
        collect_hgt(dir, &mut hgt_paths)?;
        for path in hgt_paths {
            let Some((lat0, lon0)) = parse_filename(&path) else {
                eprintln!(
                    "  (skipping {} — filename not in NxxEyyy.hgt form)",
                    path.display()
                );
                continue;
            };
            let bytes = std::fs::read(&path)
                .with_context(|| format!("reading {}", path.display()))?;
            let side = match bytes.len() {
                n if n == 1201 * 1201 * 2 => 1201,
                n if n == 3601 * 3601 * 2 => 3601,
                n => {
                    eprintln!(
                        "  (skipping {} — unexpected size {} bytes)",
                        path.display(),
                        n
                    );
                    continue;
                }
            };
            let mut data = Vec::with_capacity(side * side);
            for chunk in bytes.chunks_exact(2) {
                data.push(i16::from_be_bytes([chunk[0], chunk[1]]));
            }
            dem.tiles.insert((lat0, lon0), Tile { lat0, lon0, side, data });
        }
        Ok(dem)
    }

    pub fn elevation_at(&self, p: LatLon) -> Option<f64> {
        let lat0 = p.lat.floor() as i32;
        let lon0 = p.lon.floor() as i32;
        let tile = self.tiles.get(&(lat0, lon0))?;
        tile.elevation_at(p)
    }
}

fn collect_hgt(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    for entry in std::fs::read_dir(dir)
        .with_context(|| format!("reading {}", dir.display()))?
    {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            collect_hgt(&path, out)?;
        } else if path.extension().map(|e| e.eq_ignore_ascii_case("hgt")) == Some(true) {
            out.push(path);
        }
    }
    Ok(())
}

/// Map a viewfinderpanoramas DEM3 pack name (e.g. `L31` for the central Alps)
/// to its zip URL. Pack names are 3-character grid codes — see
/// <https://viewfinderpanoramas.org/dem3.html> for the world map.
pub fn pack_to_url(pack: &str) -> String {
    format!(
        "https://viewfinderpanoramas.org/dem3/{}.zip",
        pack.to_ascii_uppercase()
    )
}

/// Make sure each named pack has been fetched into `cache_dir`. Packs already
/// downloaded (tracked with a `.PACK.fetched` marker file) are skipped.
/// Mirrors the OSM-PBF flow in `crate::fetch`.
pub fn ensure_packs(cache_dir: &Path, packs: &[&str]) -> Result<()> {
    std::fs::create_dir_all(cache_dir)
        .with_context(|| format!("creating cache dir {}", cache_dir.display()))?;
    for pack in packs {
        let pack = pack.trim();
        if pack.is_empty() {
            continue;
        }
        ensure_pack(cache_dir, pack)?;
    }
    Ok(())
}

fn pack_marker(cache_dir: &Path, pack: &str) -> PathBuf {
    cache_dir.join(format!(".{}.fetched", pack.to_ascii_uppercase()))
}

fn ensure_pack(cache_dir: &Path, pack: &str) -> Result<()> {
    let marker = pack_marker(cache_dir, pack);
    if marker.exists() {
        eprintln!("DEM pack {pack}: already in cache");
        return Ok(());
    }
    let url = pack_to_url(pack);
    eprintln!("Downloading DEM pack {pack} from {url}");

    let resp = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(60 * 30))
        .call()
        .with_context(|| format!("requesting {url}"))?;
    if resp.status() < 200 || resp.status() >= 300 {
        return Err(anyhow!(
            "HTTP {} for {url} — check the pack name (e.g. L31 for the Alps)",
            resp.status()
        ));
    }
    let total: Option<u64> = resp
        .header("content-length")
        .and_then(|s| s.parse().ok());
    let pb = match total {
        Some(n) => {
            let pb = ProgressBar::new(n);
            pb.set_style(
                ProgressStyle::with_template(
                    "  {bar:30.cyan/blue} {bytes}/{total_bytes}  {bytes_per_sec}  ETA {eta}",
                )
                .unwrap()
                .progress_chars("=>-"),
            );
            pb
        }
        None => {
            let pb = ProgressBar::new_spinner();
            pb.set_style(
                ProgressStyle::with_template("  {spinner} {bytes} ({bytes_per_sec})").unwrap(),
            );
            pb.enable_steady_tick(std::time::Duration::from_millis(120));
            pb
        }
    };
    let zip_path = cache_dir.join(format!("{}.zip.part", pack.to_ascii_uppercase()));
    {
        let mut f = File::create(&zip_path)
            .with_context(|| format!("creating {}", zip_path.display()))?;
        let mut reader = resp.into_reader();
        let mut buf = vec![0u8; 64 * 1024];
        loop {
            let n = reader.read(&mut buf)?;
            if n == 0 {
                break;
            }
            f.write_all(&buf[..n])?;
            pb.inc(n as u64);
        }
        f.flush()?;
    }
    pb.finish_and_clear();

    eprintln!("Extracting tiles…");
    let zf = File::open(&zip_path)?;
    let mut archive = zip::ZipArchive::new(zf).context("opening zip")?;
    let mut extracted = 0usize;
    for i in 0..archive.len() {
        let mut entry = archive.by_index(i)?;
        if entry.is_dir() {
            continue;
        }
        let name = entry.name().to_string();
        if !name.to_ascii_lowercase().ends_with(".hgt") {
            continue;
        }
        // Strip directory components and any zip-slip dot segments — we only
        // want a flat NxxEyyy.hgt file in the cache dir.
        let stem = Path::new(&name)
            .file_name()
            .ok_or_else(|| anyhow!("zip entry has no filename: {name}"))?;
        let out_path = cache_dir.join(stem);
        let mut out = File::create(&out_path)
            .with_context(|| format!("creating {}", out_path.display()))?;
        std::io::copy(&mut entry, &mut out)?;
        extracted += 1;
    }
    drop(archive);
    let _ = std::fs::remove_file(&zip_path);
    eprintln!("Extracted {} .hgt tiles into {}", extracted, cache_dir.display());
    // Drop the marker so future runs skip the download.
    std::fs::write(&marker, b"")
        .with_context(|| format!("writing marker {}", marker.display()))?;
    Ok(())
}

fn parse_filename(p: &Path) -> Option<(i32, i32)> {
    let stem = p.file_stem()?.to_str()?.to_ascii_uppercase();
    if stem.len() != 7 {
        return None;
    }
    let bytes = stem.as_bytes();
    let lat_sign = match bytes[0] as char {
        'N' => 1,
        'S' => -1,
        _ => return None,
    };
    let lon_sign = match bytes[3] as char {
        'E' => 1,
        'W' => -1,
        _ => return None,
    };
    let lat: i32 = std::str::from_utf8(&bytes[1..3]).ok()?.parse().ok()?;
    let lon: i32 = std::str::from_utf8(&bytes[4..7]).ok()?.parse().ok()?;
    Some((lat_sign * lat, lon_sign * lon))
}
