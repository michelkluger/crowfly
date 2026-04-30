//! Download OSM PBF extracts from Geofabrik, with on-disk caching.

use anyhow::{anyhow, Context, Result};
use indicatif::{ProgressBar, ProgressStyle};
use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

/// Map a short or full region key to a Geofabrik URL. Examples:
/// - "switzerland" -> europe/switzerland
/// - "europe/switzerland" -> europe/switzerland
/// - "north-america/canada" -> north-america/canada
pub fn region_to_url(region: &str) -> String {
    let path = if region.contains('/') {
        region.to_string()
    } else {
        // Best-effort: assume Europe unless caller specified a continent.
        // Users can always pass an explicit "continent/country" path.
        format!("europe/{}", region)
    };
    format!("https://download.geofabrik.de/{path}-latest.osm.pbf")
}

/// Ensure a PBF for the given region key exists on disk; download if missing.
/// Returns the path to the cached file.
pub fn ensure_pbf(region: &str, cache_dir: &Path) -> Result<PathBuf> {
    std::fs::create_dir_all(cache_dir)
        .with_context(|| format!("creating cache dir {}", cache_dir.display()))?;
    let safe = region.replace('/', "_");
    let target = cache_dir.join(format!("{safe}-latest.osm.pbf"));
    if target.exists() {
        let meta = std::fs::metadata(&target)?;
        eprintln!(
            "Using cached extract: {} ({:.1} MB)",
            target.display(),
            meta.len() as f64 / 1_048_576.0
        );
        return Ok(target);
    }
    let url = region_to_url(region);
    eprintln!("Downloading {url}");

    let resp = ureq::get(&url)
        .timeout(std::time::Duration::from_secs(60 * 30))
        .call()
        .with_context(|| format!("requesting {url}"))?;
    if resp.status() < 200 || resp.status() >= 300 {
        return Err(anyhow!(
            "HTTP {} for {url} — check the region key",
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

    let tmp = target.with_extension("pbf.part");
    let mut f = File::create(&tmp).with_context(|| format!("creating {}", tmp.display()))?;
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
    drop(f);
    std::fs::rename(&tmp, &target)?;
    pb.finish_and_clear();
    eprintln!("Saved: {}", target.display());
    Ok(target)
}
