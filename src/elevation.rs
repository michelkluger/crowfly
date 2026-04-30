//! Elevation lookup via OpenTopoData (SRTM 30 m), with on-disk caching.
//!
//! For each route we sample N points evenly along the path, batch up to 100
//! per HTTP request, and cache results in `./elevation-cache/cache.json`
//! keyed by rounded lat/lon. A failed fetch degrades gracefully — callers
//! get `None` and the UI just hides the elevation panel.

use crate::geodesy::haversine;
use crate::geodesy::LatLon;
use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Per-route elevation profile.
#[derive(Debug, Clone, Serialize)]
pub struct Profile {
    /// Sampled elevations in metres, equally spaced along the route by arc length.
    pub samples_m: Vec<f64>,
    pub max_m: f64,
    pub min_m: f64,
    /// Cumulative ascent / descent in metres, from sample-to-sample diffs.
    pub ascent_m: f64,
    pub descent_m: f64,
}

#[derive(Default, Serialize, Deserialize)]
struct Cache {
    /// Key format: "{lat:.4},{lon:.4}".
    points: HashMap<String, f64>,
}

pub struct ElevationClient {
    cache_path: PathBuf,
    cache: Cache,
    dirty: bool,
    /// If false, suppress further requests after a network failure so we don't
    /// hammer the API for every subsequent route.
    online: bool,
}

impl ElevationClient {
    pub fn new(cache_dir: &Path) -> Result<Self> {
        fs::create_dir_all(cache_dir)?;
        let cache_path = cache_dir.join("cache.json");
        let cache: Cache = if cache_path.exists() {
            let s = fs::read_to_string(&cache_path)?;
            serde_json::from_str(&s).unwrap_or_default()
        } else {
            Cache::default()
        };
        Ok(Self {
            cache_path,
            cache,
            dirty: false,
            online: true,
        })
    }

    pub fn save(&self) -> Result<()> {
        if !self.dirty {
            return Ok(());
        }
        let s = serde_json::to_string(&self.cache)?;
        fs::write(&self.cache_path, s).context("writing elevation cache")?;
        Ok(())
    }

    /// Return a profile for the route, with `n_samples` samples evenly spaced
    /// by arc length. Returns None if the route is empty or all lookups fail.
    pub fn profile(&mut self, route: &[LatLon], n_samples: usize) -> Option<Profile> {
        if route.len() < 2 {
            return None;
        }
        let n = n_samples.max(8);
        let samples = sample_along(route, n);
        let elevations = self.batch_lookup(&samples).ok()?;
        if elevations.iter().all(|e| e == &0.0) {
            // Likely all-fallback or missing data; treat as unavailable.
            return None;
        }
        let mut min_m = f64::INFINITY;
        let mut max_m = f64::NEG_INFINITY;
        for &e in &elevations {
            if e < min_m {
                min_m = e;
            }
            if e > max_m {
                max_m = e;
            }
        }
        let mut ascent = 0.0;
        let mut descent = 0.0;
        for w in elevations.windows(2) {
            let d = w[1] - w[0];
            if d > 0.0 {
                ascent += d;
            } else {
                descent -= d;
            }
        }
        Some(Profile {
            samples_m: elevations,
            max_m,
            min_m,
            ascent_m: ascent,
            descent_m: descent,
        })
    }

    fn batch_lookup(&mut self, points: &[LatLon]) -> Result<Vec<f64>> {
        let mut out = vec![f64::NAN; points.len()];
        let mut to_fetch: Vec<(usize, LatLon)> = Vec::new();
        for (i, p) in points.iter().enumerate() {
            let key = cache_key(*p);
            if let Some(v) = self.cache.points.get(&key) {
                out[i] = *v;
            } else {
                to_fetch.push((i, *p));
            }
        }
        if !self.online || to_fetch.is_empty() {
            return Ok(out
                .into_iter()
                .map(|v| if v.is_nan() { 0.0 } else { v })
                .collect());
        }

        // OpenTopoData SRTM30m accepts up to 100 locations per request.
        for chunk in to_fetch.chunks(100) {
            match self.fetch_one(chunk.iter().map(|(_, p)| *p).collect::<Vec<_>>().as_slice()) {
                Ok(elevs) => {
                    for ((i, p), e) in chunk.iter().zip(elevs.iter()) {
                        out[*i] = *e;
                        self.cache.points.insert(cache_key(*p), *e);
                    }
                    self.dirty = true;
                }
                Err(err) => {
                    eprintln!("  (elevation lookup failed: {err}; skipping further requests)");
                    self.online = false;
                    break;
                }
            }
            // Be polite to the public endpoint.
            std::thread::sleep(Duration::from_millis(1100));
        }

        Ok(out
            .into_iter()
            .map(|v| if v.is_nan() { 0.0 } else { v })
            .collect())
    }

    fn fetch_one(&self, points: &[LatLon]) -> Result<Vec<f64>> {
        let locs = points
            .iter()
            .map(|p| format!("{:.5},{:.5}", p.lat, p.lon))
            .collect::<Vec<_>>()
            .join("|");
        let url = format!(
            "https://api.opentopodata.org/v1/srtm30m?locations={}",
            locs
        );
        let resp = ureq::get(&url)
            .timeout(Duration::from_secs(30))
            .call()
            .with_context(|| "GET opentopodata".to_string())?;
        if resp.status() < 200 || resp.status() >= 300 {
            return Err(anyhow!("HTTP {} from opentopodata", resp.status()));
        }
        let body = resp.into_string()?;
        let v: Value = serde_json::from_str(&body)?;
        let results: &Vec<Value> = v
            .get("results")
            .and_then(|x: &Value| x.as_array())
            .ok_or_else(|| anyhow!("unexpected response shape"))?;
        if results.len() != points.len() {
            return Err(anyhow!(
                "got {} results, expected {}",
                results.len(),
                points.len()
            ));
        }
        Ok(results
            .iter()
            .map(|r: &Value| {
                r.get("elevation")
                    .and_then(|e: &Value| e.as_f64())
                    .unwrap_or(0.0)
            })
            .collect())
    }
}

fn cache_key(p: LatLon) -> String {
    format!("{:.4},{:.4}", p.lat, p.lon)
}

/// Sample `n` points evenly along the polyline by arc length.
pub fn sample_along(points: &[LatLon], n: usize) -> Vec<LatLon> {
    if points.len() < 2 || n < 2 {
        return points.to_vec();
    }
    let mut cum = Vec::with_capacity(points.len());
    cum.push(0.0);
    for w in points.windows(2) {
        let d = haversine(w[0], w[1]);
        cum.push(cum.last().unwrap() + d);
    }
    let total = *cum.last().unwrap();
    if total == 0.0 {
        return vec![points[0]; n];
    }
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let t = (i as f64) / ((n - 1) as f64);
        let target = t * total;
        // Linear scan; n is small.
        let mut j = 0;
        while j + 1 < cum.len() && cum[j + 1] < target {
            j += 1;
        }
        let d0 = cum[j];
        let d1 = cum[(j + 1).min(cum.len() - 1)];
        let frac = if d1 > d0 { (target - d0) / (d1 - d0) } else { 0.0 };
        let p0 = points[j];
        let p1 = points[(j + 1).min(points.len() - 1)];
        out.push(LatLon::new(
            p0.lat + (p1.lat - p0.lat) * frac,
            p0.lon + (p1.lon - p0.lon) * frac,
        ));
    }
    out
}
