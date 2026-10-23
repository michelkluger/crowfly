//! Elevation profiles along routed polylines, from one of two sources:
//!
//! - [`profile_from_dem`] samples the in-memory SRTM grid (`crate::dem`)
//!   every [`PROFILE_STEP_M`] metres. `serve` already loads that grid for
//!   slope-aware routing, so profiles cost no network and no rate limit.
//! - [`ElevationClient`] queries OpenTopoData (SRTM 30 m) over HTTP, batching
//!   up to 100 points per request and caching results in
//!   `./elevation-cache/cache.json` keyed by rounded lat/lon. Used by the
//!   `plan` / `explore` CLIs, which don't load a DEM.
//!
//! Either way a failed or incomplete lookup degrades gracefully — callers get
//! `None` and the UI just hides the elevation panel.

use crate::dem::Dem;
use crate::geodesy::haversine;
use crate::geodesy::LatLon;
use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Spacing of DEM samples along a route. The SRTM3 grid is ~90 m × ~65 m at
/// Swiss latitudes, so 50 m visits every cell a route crosses; on Swiss test
/// lines, ascent totals agree within ~2 % between 25 m and 100 m steps.
pub const PROFILE_STEP_M: f64 = 50.0;

/// Climbs and drops smaller than this are treated as DEM noise when
/// accumulating ascent/descent. Without it, 50 m sampling across flat Swiss
/// Plateau farmland reports ~85 m of climbing per 10 km; 5 m roughly halves
/// that while trimming real climbs by under 10 %.
pub const CLIMB_THRESHOLD_M: f64 = 5.0;

/// Cap on samples per route (20 000 × 50 m = 1 000 km).
const MAX_DENSE_SAMPLES: usize = 20_000;

/// A profile is dropped when more than this share of its samples has no data
/// (route leaves DEM coverage, failed lookups). Smaller gaps are bridged with
/// the nearest valid sample instead of reading as 0 m.
const MAX_MISSING_FRACTION: f64 = 0.1;

/// Per-route elevation profile.
#[derive(Debug, Clone, Serialize)]
pub struct Profile {
    /// Elevations in metres, equally spaced along the route by arc length.
    /// Chart resolution — ascent/descent/min/max use every underlying sample.
    pub samples_m: Vec<f64>,
    pub max_m: f64,
    pub min_m: f64,
    /// Cumulative ascent / descent in metres, ignoring wiggles below
    /// [`CLIMB_THRESHOLD_M`].
    pub ascent_m: f64,
    pub descent_m: f64,
}

impl Profile {
    /// Build a profile from elevations sampled at equal arc-length spacing.
    /// `None` marks a sample without data. `n_display` is the length of
    /// `samples_m`.
    fn from_samples(samples: &[Option<f64>], n_display: usize) -> Option<Profile> {
        let zs = fill_gaps(samples)?;
        let mut min_m = f64::INFINITY;
        let mut max_m = f64::NEG_INFINITY;
        for &z in &zs {
            min_m = min_m.min(z);
            max_m = max_m.max(z);
        }
        let (ascent_m, descent_m) = climb_totals(&zs, CLIMB_THRESHOLD_M);
        Some(Profile {
            samples_m: resample(&zs, n_display),
            max_m,
            min_m,
            ascent_m,
            descent_m,
        })
    }
}

/// Profile of `route` from the resident DEM, with `n_samples` chart points.
/// Returns None if the route is empty or mostly outside DEM coverage.
pub fn profile_from_dem(dem: &Dem, route: &[LatLon], n_samples: usize) -> Option<Profile> {
    if route.len() < 2 {
        return None;
    }
    let length_m: f64 = route.windows(2).map(|w| haversine(w[0], w[1])).sum();
    let n_dense = ((length_m / PROFILE_STEP_M).ceil() as usize + 1).clamp(2, MAX_DENSE_SAMPLES);
    let samples: Vec<Option<f64>> = sample_along(route, n_dense)
        .into_iter()
        .map(|p| dem.elevation_at(p))
        .collect();
    Profile::from_samples(&samples, n_samples.max(8))
}

/// Replace missing samples with the nearest preceding valid one (leading
/// gaps take the first valid value). None if too much is missing.
fn fill_gaps(samples: &[Option<f64>]) -> Option<Vec<f64>> {
    let missing = samples.iter().filter(|s| s.is_none()).count();
    if missing as f64 > MAX_MISSING_FRACTION * samples.len() as f64 {
        return None;
    }
    let mut last = samples.iter().flatten().next().copied()?;
    Some(
        samples
            .iter()
            .map(|s| {
                if let Some(z) = s {
                    last = *z;
                }
                last
            })
            .collect(),
    )
}

/// Hysteresis ascent/descent: only count a change once it moves at least
/// `threshold_m` away from the last counted elevation.
fn climb_totals(zs: &[f64], threshold_m: f64) -> (f64, f64) {
    let Some(&first) = zs.first() else {
        return (0.0, 0.0);
    };
    let mut anchor = first;
    let mut ascent = 0.0;
    let mut descent = 0.0;
    for &z in &zs[1..] {
        let d = z - anchor;
        if d >= threshold_m {
            ascent += d;
            anchor = z;
        } else if -d >= threshold_m {
            descent -= d;
            anchor = z;
        }
    }
    (ascent, descent)
}

/// Linearly resample an evenly spaced series to `n` evenly spaced values.
fn resample(zs: &[f64], n: usize) -> Vec<f64> {
    if zs.len() < 2 || n < 2 {
        return zs.to_vec();
    }
    let last = (zs.len() - 1) as f64;
    (0..n)
        .map(|i| {
            let x = i as f64 * last / (n - 1) as f64;
            let j = (x.floor() as usize).min(zs.len() - 2);
            let t = x - j as f64;
            zs[j] + (zs[j + 1] - zs[j]) * t
        })
        .collect()
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
    /// by arc length. Returns None if the route is empty or too many lookups
    /// fail.
    pub fn profile(&mut self, route: &[LatLon], n_samples: usize) -> Option<Profile> {
        if route.len() < 2 {
            return None;
        }
        let n = n_samples.max(8);
        let samples = sample_along(route, n);
        let elevations = self.batch_lookup(&samples).ok()?;
        Profile::from_samples(&elevations, n)
    }

    fn batch_lookup(&mut self, points: &[LatLon]) -> Result<Vec<Option<f64>>> {
        let mut out = vec![None; points.len()];
        let mut to_fetch: Vec<(usize, LatLon)> = Vec::new();
        for (i, p) in points.iter().enumerate() {
            let key = cache_key(*p);
            if let Some(v) = self.cache.points.get(&key) {
                out[i] = Some(*v);
            } else {
                to_fetch.push((i, *p));
            }
        }
        if !self.online || to_fetch.is_empty() {
            return Ok(out);
        }

        // OpenTopoData SRTM30m accepts up to 100 locations per request.
        for chunk in to_fetch.chunks(100) {
            match self.fetch_one(chunk.iter().map(|(_, p)| *p).collect::<Vec<_>>().as_slice()) {
                Ok(elevs) => {
                    for ((i, p), e) in chunk.iter().zip(elevs.iter()) {
                        out[*i] = *e;
                        if let Some(z) = e {
                            self.cache.points.insert(cache_key(*p), *z);
                        }
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

        Ok(out)
    }

    fn fetch_one(&self, points: &[LatLon]) -> Result<Vec<Option<f64>>> {
        let locs = points
            .iter()
            .map(|p| format!("{:.5},{:.5}", p.lat, p.lon))
            .collect::<Vec<_>>()
            .join("|");
        let url = format!("https://api.opentopodata.org/v1/srtm30m?locations={}", locs);
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
            .map(|r: &Value| r.get("elevation").and_then(|e: &Value| e.as_f64()))
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
    // Targets increase monotonically, so the segment cursor only moves
    // forward: O(points + n) even for dense DEM sampling.
    let mut j = 0;
    for i in 0..n {
        let t = (i as f64) / ((n - 1) as f64);
        let target = t * total;
        while j + 1 < cum.len() && cum[j + 1] < target {
            j += 1;
        }
        let d0 = cum[j];
        let d1 = cum[(j + 1).min(cum.len() - 1)];
        let frac = if d1 > d0 {
            (target - d0) / (d1 - d0)
        } else {
            0.0
        };
        let p0 = points[j];
        let p1 = points[(j + 1).min(points.len() - 1)];
        out.push(LatLon::new(
            p0.lat + (p1.lat - p0.lat) * frac,
            p0.lon + (p1.lon - p0.lon) * frac,
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::Tile;

    /// One-tile DEM over N46E007 that rises `per_row_m` per grid row towards
    /// the north (row 0 is the north edge), so north-bound routes climb.
    fn ramp_dem(side: usize, per_row_m: f64) -> Dem {
        let mut data = Vec::with_capacity(side * side);
        for row in 0..side {
            for _ in 0..side {
                data.push((2000.0 - row as f64 * per_row_m) as i16);
            }
        }
        let mut dem = Dem::default();
        dem.tiles.insert(
            (46, 7),
            Tile {
                lat0: 46,
                lon0: 7,
                side,
                data,
            },
        );
        dem
    }

    #[test]
    fn climb_totals_ignores_sub_threshold_noise() {
        // ±2 m jitter on flat ground, then a clean 100 m climb and 40 m drop.
        let mut zs = vec![500.0, 502.0, 498.0, 501.0, 499.0, 500.0];
        zs.extend((1..=10).map(|i| 500.0 + 10.0 * i as f64));
        zs.extend((1..=4).map(|i| 600.0 - 10.0 * i as f64));
        let (asc, desc) = climb_totals(&zs, 5.0);
        assert!((asc - 100.0).abs() < 1e-9, "ascent {asc}");
        assert!((desc - 40.0).abs() < 1e-9, "descent {desc}");
        let (raw_asc, _) = climb_totals(&zs, 0.0);
        assert!(raw_asc > asc, "zero threshold should count the jitter");
    }

    #[test]
    fn fill_gaps_bridges_small_gaps_and_rejects_large_ones() {
        let mut s: Vec<Option<f64>> = (0..20).map(|i| Some(i as f64)).collect();
        s[0] = None;
        s[10] = None;
        assert_eq!(
            fill_gaps(&s).unwrap()[0],
            1.0,
            "leading gap takes first valid"
        );
        assert_eq!(fill_gaps(&s).unwrap()[10], 9.0, "inner gap carries forward");
        s[11] = None;
        assert!(fill_gaps(&s).is_none(), "15 % missing exceeds the cap");
        assert!(fill_gaps(&[None, None]).is_none());
    }

    #[test]
    fn missing_samples_never_read_as_sea_level() {
        // Regression: OpenTopoData misses used to become 0.0 m, turning a
        // flat 500 m route into a 1000 m "climb".
        let mut s = vec![Some(500.0); 30];
        s[15] = None;
        let p = Profile::from_samples(&s, 30).unwrap();
        assert_eq!(p.min_m, 500.0);
        assert_eq!(p.ascent_m, 0.0);
    }

    #[test]
    fn resample_keeps_endpoints_and_interpolates() {
        let zs = [0.0, 10.0, 20.0, 30.0, 40.0];
        assert_eq!(resample(&zs, 3), vec![0.0, 20.0, 40.0]);
        assert_eq!(resample(&zs, 5), zs.to_vec());
        assert_eq!(resample(&zs, 9)[1], 5.0);
    }

    #[test]
    fn sample_along_is_evenly_spaced_with_exact_endpoints() {
        let route = [
            LatLon::new(46.5, 7.1),
            LatLon::new(46.5, 7.2),
            LatLon::new(46.6, 7.2),
        ];
        let s = sample_along(&route, 11);
        assert_eq!(s.len(), 11);
        assert_eq!(s[0], route[0]);
        assert!(haversine(s[10], route[2]) < 1e-6);
        let gaps: Vec<f64> = s.windows(2).map(|w| haversine(w[0], w[1])).collect();
        let (lo, hi) = gaps.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &g| {
            (lo.min(g), hi.max(g))
        });
        // The corner sample cuts across the bend, so allow a little slack.
        assert!(hi - lo < 0.3 * hi, "uneven spacing: {lo:.0}..{hi:.0} m");
    }

    #[test]
    fn dem_profile_measures_a_climb_densely() {
        // SRTM3-sized tile, 1 m per row (~92 m): an ~11 km north-bound line
        // crosses 120 rows, so it climbs 120 m (minus < 5 m of hysteresis).
        let dem = ramp_dem(1201, 1.0);
        let route = [LatLon::new(46.3, 7.5), LatLon::new(46.4, 7.5)];
        let p = profile_from_dem(&dem, &route, 30).expect("profile");
        assert_eq!(p.samples_m.len(), 30);
        let expected = 0.1 * 1200.0; // 0.1° of latitude = 120 rows
        assert!((p.ascent_m - expected).abs() < 6.0, "ascent {}", p.ascent_m);
        assert!(p.descent_m < 1.0, "descent {}", p.descent_m);
        assert!(p.samples_m.windows(2).all(|w| w[1] >= w[0]));
    }

    #[test]
    fn dem_profile_none_outside_coverage() {
        let dem = ramp_dem(1201, 1.0);
        let route = [LatLon::new(47.3, 8.5), LatLon::new(47.4, 8.5)];
        assert!(profile_from_dem(&dem, &route, 30).is_none());
    }
}
