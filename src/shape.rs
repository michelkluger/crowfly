//! Closed-loop shape geometry placed on the sphere.
//!
//! Each shape is described in unit space (characteristic radius = 1, +x = east,
//! +y = north, closed: last vertex == first), then projected onto the spherical
//! earth around a chosen center via `geodesy::destination` so leg distances are
//! correct at country scale rather than slightly off via flat-earth math.
//!
//! `place_shape` solves for the radius that yields the requested *perimeter*,
//! since "I want a 50 km loop" is the natural mental model.

use crate::geodesy::{destination, LatLon};

const CIRCLE_SEGMENTS: usize = 16;
const SMOOTH_CURVE_SEGMENTS: usize = 32;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ShapeKind {
    Circle,
    Triangle,
    Square,
    Pentagon,
    Hexagon,
    Star,
    Heart,
    FigureEight,
    SwissCross,
}

impl ShapeKind {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s.to_ascii_lowercase().as_str() {
            "circle" => Self::Circle,
            "triangle" => Self::Triangle,
            "square" => Self::Square,
            "pentagon" => Self::Pentagon,
            "hexagon" => Self::Hexagon,
            "star" => Self::Star,
            "heart" => Self::Heart,
            "figure-8" | "figure8" | "figureeight" | "infinity" => Self::FigureEight,
            "swiss-cross" | "swisscross" | "cross" | "swiss" => Self::SwissCross,
            _ => return None,
        })
    }
}

fn regular_polygon(n: usize) -> Vec<(f64, f64)> {
    // Start the first vertex at the top (+y), so a non-rotated triangle points
    // north — matches geographic intuition.
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let theta = (i as f64) * 2.0 * std::f64::consts::PI / (n as f64)
            + std::f64::consts::FRAC_PI_2;
        out.push((theta.cos(), theta.sin()));
    }
    out
}

fn star(points: usize, inner_ratio: f64) -> Vec<(f64, f64)> {
    let m = points * 2;
    let mut out = Vec::with_capacity(m + 1);
    for i in 0..=m {
        let r = if i % 2 == 0 { 1.0 } else { inner_ratio };
        let theta =
            (i as f64) * std::f64::consts::PI / (points as f64) + std::f64::consts::FRAC_PI_2;
        out.push((r * theta.cos(), r * theta.sin()));
    }
    out
}

fn heart(n: usize) -> Vec<(f64, f64)> {
    // Classic parametric heart. Normalised so max radius is 1.
    let mut raw: Vec<(f64, f64)> = (0..=n)
        .map(|i| {
            let t = (i as f64) * 2.0 * std::f64::consts::PI / (n as f64);
            let x = 16.0 * t.sin().powi(3);
            let y = 13.0 * t.cos()
                - 5.0 * (2.0 * t).cos()
                - 2.0 * (3.0 * t).cos()
                - (4.0 * t).cos();
            (x, y)
        })
        .collect();
    let r_max = raw
        .iter()
        .map(|p| (p.0 * p.0 + p.1 * p.1).sqrt())
        .fold(0.0_f64, f64::max)
        .max(1e-9);
    for p in &mut raw {
        p.0 /= r_max;
        p.1 /= r_max;
    }
    raw
}

fn figure_eight(n: usize) -> Vec<(f64, f64)> {
    // Lemniscate of Gerono: x = sin(t), y = sin(t) cos(t). Smooth ∞.
    (0..=n)
        .map(|i| {
            let t = (i as f64) * 2.0 * std::f64::consts::PI / (n as f64);
            (t.sin(), t.sin() * t.cos())
        })
        .collect()
}

fn swiss_cross() -> Vec<(f64, f64)> {
    // 12-vertex plus sign with arm thickness 1/3 of total span. Closed.
    let a = 1.0 / 3.0;
    vec![
        (-a, 1.0),
        (a, 1.0),
        (a, a),
        (1.0, a),
        (1.0, -a),
        (a, -a),
        (a, -1.0),
        (-a, -1.0),
        (-a, -a),
        (-1.0, -a),
        (-1.0, a),
        (-a, a),
        (-a, 1.0),
    ]
}

/// Largest distance from origin to any vertex (in unit space). Used by the
/// country-search sampler to inset the bbox so the shape doesn't escape it.
pub fn unit_max_radius(kind: ShapeKind) -> f64 {
    unit_vertices(kind)
        .iter()
        .map(|p| (p.0 * p.0 + p.1 * p.1).sqrt())
        .fold(0.0_f64, f64::max)
}

/// Perimeter of the unit-radius shape (sum of leg lengths). Used to invert
/// "I want a perimeter of P km" into the radius the shape needs.
pub fn unit_shape_perimeter(kind: ShapeKind) -> f64 {
    unit_perimeter(&unit_vertices(kind))
}

fn unit_vertices(kind: ShapeKind) -> Vec<(f64, f64)> {
    match kind {
        ShapeKind::Circle => regular_polygon(CIRCLE_SEGMENTS),
        ShapeKind::Triangle => regular_polygon(3),
        ShapeKind::Square => regular_polygon(4),
        ShapeKind::Pentagon => regular_polygon(5),
        ShapeKind::Hexagon => regular_polygon(6),
        ShapeKind::Star => star(5, 0.4),
        ShapeKind::Heart => heart(SMOOTH_CURVE_SEGMENTS),
        ShapeKind::FigureEight => figure_eight(SMOOTH_CURVE_SEGMENTS * 2),
        ShapeKind::SwissCross => swiss_cross(),
    }
}

fn unit_perimeter(verts: &[(f64, f64)]) -> f64 {
    let mut total = 0.0;
    for w in verts.windows(2) {
        let dx = w[1].0 - w[0].0;
        let dy = w[1].1 - w[0].1;
        total += (dx * dx + dy * dy).sqrt();
    }
    total
}

/// Place a closed shape on the sphere centred on `center`, sized so its
/// (unit-space) perimeter equals `perimeter_km`. `rotation_deg` rotates the
/// shape clockwise as seen from above (so 0° = pointing north, 90° = east).
///
/// Returns `n+1` vertices with `verts[0] == verts[n]` so consecutive-pair
/// iteration walks the closed loop without a special case at the wrap.
pub fn place_shape(
    center: LatLon,
    kind: ShapeKind,
    perimeter_km: f64,
    rotation_deg: f64,
) -> Vec<LatLon> {
    let unit = unit_vertices(kind);
    let unit_perim = unit_perimeter(&unit).max(1e-9);
    let radius_m = (perimeter_km * 1000.0) / unit_perim;
    let rot_rad = rotation_deg.to_radians();
    let mut out = Vec::with_capacity(unit.len());
    for (x, y) in unit {
        let r = (x * x + y * y).sqrt();
        if r < 1e-9 {
            out.push(center);
            continue;
        }
        let theta = y.atan2(x); // CCW from east, radians
        // Geographic bearing (CW from north) = π/2 − θ + rotation.
        let bearing = std::f64::consts::FRAC_PI_2 - theta + rot_rad;
        let dist = r * radius_m;
        out.push(destination(center, bearing, dist));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::haversine;

    #[test]
    fn circle_perimeter_matches_request() {
        let verts = place_shape(LatLon::new(46.8, 8.2), ShapeKind::Circle, 50.0, 0.0);
        let p_m: f64 = verts
            .windows(2)
            .map(|w| haversine(w[0], w[1]))
            .sum();
        assert!(
            (p_m - 50_000.0).abs() < 1_500.0,
            "circle perimeter {p_m:.0} m far from 50 000 m"
        );
        // Closed loop.
        let first = verts[0];
        let last = verts[verts.len() - 1];
        assert!(haversine(first, last) < 5.0);
    }

    #[test]
    fn triangle_has_three_legs_plus_close() {
        let verts = place_shape(LatLon::new(46.8, 8.2), ShapeKind::Triangle, 60.0, 0.0);
        assert_eq!(verts.len(), 4); // 3 vertices + close
    }

    #[test]
    fn rotation_changes_first_vertex_bearing() {
        let center = LatLon::new(46.8, 8.2);
        let v0 = place_shape(center, ShapeKind::Triangle, 60.0, 0.0)[0];
        let v90 = place_shape(center, ShapeKind::Triangle, 60.0, 90.0)[0];
        // 0° → first vertex due north (Δlat > 0, Δlon ≈ 0).
        assert!(v0.lat > center.lat);
        assert!((v0.lon - center.lon).abs() < 0.05);
        // 90° → first vertex due east (Δlat ≈ 0, Δlon > 0).
        assert!((v90.lat - center.lat).abs() < 0.05);
        assert!(v90.lon > center.lon);
    }

    #[test]
    fn parse_kind_accepts_aliases() {
        assert_eq!(ShapeKind::parse("circle"), Some(ShapeKind::Circle));
        assert_eq!(ShapeKind::parse("Heart"), Some(ShapeKind::Heart));
        assert_eq!(ShapeKind::parse("figure-8"), Some(ShapeKind::FigureEight));
        assert_eq!(ShapeKind::parse("swiss"), Some(ShapeKind::SwissCross));
        assert_eq!(ShapeKind::parse("blob"), None);
    }
}
