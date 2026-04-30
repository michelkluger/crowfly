//! Geodesic primitives on a spherical earth.
//!
//! We avoid pulling in `proj` or full ellipsoid math: for a corridor a few km
//! wide on country-scale lines, spherical-earth haversine + cross-track distance
//! is accurate to well under a metre. All angles are in degrees on input/output
//! and radians internally. Distances are in metres.

const R_EARTH: f64 = 6_371_008.8; // mean earth radius, metres (IUGG)

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

impl LatLon {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }
}

fn to_rad(d: f64) -> f64 {
    d.to_radians()
}

/// Great-circle distance between two points, metres.
pub fn haversine(a: LatLon, b: LatLon) -> f64 {
    let phi1 = to_rad(a.lat);
    let phi2 = to_rad(b.lat);
    let dphi = to_rad(b.lat - a.lat);
    let dlam = to_rad(b.lon - a.lon);
    let h = (dphi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (dlam / 2.0).sin().powi(2);
    2.0 * R_EARTH * h.sqrt().asin()
}

/// Initial bearing from a to b, in radians, range (-pi, pi].
pub fn bearing_rad(a: LatLon, b: LatLon) -> f64 {
    let phi1 = to_rad(a.lat);
    let phi2 = to_rad(b.lat);
    let dlam = to_rad(b.lon - a.lon);
    let y = dlam.sin() * phi2.cos();
    let x = phi1.cos() * phi2.sin() - phi1.sin() * phi2.cos() * dlam.cos();
    y.atan2(x)
}

/// Signed cross-track distance (metres) of point `p` from the great circle
/// going from `a` toward `b`. Positive if `p` is to the right of the path
/// when travelling a -> b.
pub fn cross_track(p: LatLon, a: LatLon, b: LatLon) -> f64 {
    let d_ap = haversine(a, p) / R_EARTH;
    let theta_ap = bearing_rad(a, p);
    let theta_ab = bearing_rad(a, b);
    (d_ap.sin() * (theta_ap - theta_ab).sin()).asin() * R_EARTH
}

/// Along-track distance (metres) of point `p` measured along the great circle
/// from `a` toward `b`. May be negative (before a) or > |ab| (past b).
pub fn along_track(p: LatLon, a: LatLon, b: LatLon) -> f64 {
    let d_ap = haversine(a, p) / R_EARTH;
    let xt = cross_track(p, a, b) / R_EARTH;
    // Guard against domain errors when p lies almost on the line.
    let cos_at = (d_ap.cos() / xt.cos()).clamp(-1.0, 1.0);
    let sign = {
        let theta_ap = bearing_rad(a, p);
        let theta_ab = bearing_rad(a, b);
        let mut delta = theta_ap - theta_ab;
        while delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        if delta.abs() > std::f64::consts::FRAC_PI_2 {
            -1.0
        } else {
            1.0
        }
    };
    sign * cos_at.acos() * R_EARTH
}

/// Move from `a` along bearing (radians) by `dist` metres.
pub fn destination(a: LatLon, bearing: f64, dist: f64) -> LatLon {
    let phi1 = to_rad(a.lat);
    let lam1 = to_rad(a.lon);
    let dr = dist / R_EARTH;
    let phi2 = (phi1.sin() * dr.cos() + phi1.cos() * dr.sin() * bearing.cos()).asin();
    let lam2 = lam1
        + (bearing.sin() * dr.sin() * phi1.cos())
            .atan2(dr.cos() - phi1.sin() * phi2.sin());
    LatLon::new(phi2.to_degrees(), lam2.to_degrees())
}

/// Sample the geodesic from `a` to `b` at roughly `step_m` metre spacing.
/// Always includes both endpoints.
pub fn sample_line(a: LatLon, b: LatLon, step_m: f64) -> Vec<LatLon> {
    let total = haversine(a, b);
    if total < step_m {
        return vec![a, b];
    }
    let n = (total / step_m).ceil() as usize;
    let bearing = bearing_rad(a, b);
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let d = total * (i as f64) / (n as f64);
        out.push(destination(a, bearing, d));
    }
    out
}

/// Build the four-corner polygon (closed ring) of a corridor of total
/// `width_m` around the great circle from `a` to `b`. Sampled along the line
/// to follow curvature; works fine for sub-thousand-km lines.
pub fn corridor_polygon(a: LatLon, b: LatLon, width_m: f64) -> Vec<LatLon> {
    let half = width_m / 2.0;
    let samples = sample_line(a, b, 5_000.0);
    let mut left = Vec::with_capacity(samples.len());
    let mut right = Vec::with_capacity(samples.len());
    for w in samples.windows(2) {
        let bearing = bearing_rad(w[0], w[1]);
        let perp_left = bearing - std::f64::consts::FRAC_PI_2;
        let perp_right = bearing + std::f64::consts::FRAC_PI_2;
        left.push(destination(w[0], perp_left, half));
        right.push(destination(w[0], perp_right, half));
    }
    // Close the cap at the far end using the last bearing.
    let last_bearing = bearing_rad(samples[samples.len() - 2], samples[samples.len() - 1]);
    left.push(destination(
        *samples.last().unwrap(),
        last_bearing - std::f64::consts::FRAC_PI_2,
        half,
    ));
    right.push(destination(
        *samples.last().unwrap(),
        last_bearing + std::f64::consts::FRAC_PI_2,
        half,
    ));
    let mut ring = Vec::with_capacity(left.len() + right.len() + 1);
    ring.extend(left);
    ring.extend(right.into_iter().rev());
    ring.push(ring[0]);
    ring
}

/// True if `p` is inside the corridor of the given half-width around segment a -> b.
pub fn in_corridor(p: LatLon, a: LatLon, b: LatLon, half_width_m: f64, total_len_m: f64) -> bool {
    let xt = cross_track(p, a, b).abs();
    if xt > half_width_m {
        return false;
    }
    let at = along_track(p, a, b);
    at >= -half_width_m && at <= total_len_m + half_width_m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn haversine_known() {
        // Geneva -> Zurich is roughly 224 km
        let geneva = LatLon::new(46.2044, 6.1432);
        let zurich = LatLon::new(47.3769, 8.5417);
        let d = haversine(geneva, zurich);
        assert!((d - 224_000.0).abs() < 5_000.0, "got {d}");
    }

    #[test]
    fn cross_track_zero_on_line() {
        let a = LatLon::new(46.0, 8.0);
        let b = LatLon::new(48.0, 8.0);
        let mid = LatLon::new(47.0, 8.0);
        // mid is on the meridian, cross-track should be small (not exactly 0
        // because the meridian on a sphere needs azimuth tracking, but close)
        assert!(cross_track(mid, a, b).abs() < 100.0);
    }

    #[test]
    fn corridor_predicate() {
        let a = LatLon::new(46.17, 8.79); // Locarno
        let b = LatLon::new(47.68, 8.62); // Schaffhausen
        let total = haversine(a, b);
        // a point near the line
        let p_near = LatLon::new(47.0, 8.7);
        assert!(in_corridor(p_near, a, b, 10_000.0, total));
        // a point far off
        let p_far = LatLon::new(47.0, 6.0);
        assert!(!in_corridor(p_far, a, b, 10_000.0, total));
    }
}
