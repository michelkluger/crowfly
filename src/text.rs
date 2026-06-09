//! Text-writing mode.
//!
//! Lay out an ASCII string as Hershey strokes along a user-defined baseline
//! (two clicks: anchor + direction), then route each stroke through the OSM
//! graph. Per-stroke routing is independent so the server-side handler farms
//! the strokes out across rayon's pool.
//!
//! # Why this looks different from the first attempt
//!
//! An earlier version (see `git stash`) routed every consecutive Hershey
//! control point as its own leg, then drew the un-routed strokes as a bold
//! overlay so the user could "read through" the routed mess. This module
//! takes a different stance: **the routed output is the deliverable**. Three
//! things make that work:
//!
//! 1. **RDP simplification of each Hershey stroke** before routing. A simplex
//!    'O' has 13+ control points; collapsing to ~4 makes each leg long enough
//!    that snap noise (the displacement from a clicked point to the nearest
//!    graph node) is small relative to leg length.
//! 2. **Tight, stroke-scoped corridor**: half-width ≈ 10 % of the letter
//!    height, α=30. Letters need shape fidelity, not road preference.
//! 3. **Detour-cap drop**: strokes whose routed length blows past the
//!    intended length by more than ~60 % are omitted. A missing stem on an
//!    'A' is more readable than a routed cloud where the stem should be.
//!
//! Hershey font-data convention (matches `src/hershey_fonts.rs`): +y is UP on
//! the page, so glyph control points have NEGATIVE y for parts above the
//! baseline. The baseline of a letter sits at y ≈ −cap_height; the letter top
//! at y ≈ 0. We translate by +cap_height when projecting so y_above_baseline
//! is non-negative.

use crate::geodesy::{destination, haversine, LatLon};
use crate::hershey_fonts::{Font, Glyph};
use crate::osm::{BBox, EdgeData, Graph};
use crate::route::{self, RouteParams};
use crate::viability::NoGoZone;

/// Vertical spacing between successive baselines, as a multiple of cap height.
/// 1.5× keeps descenders on one line clear of ascenders on the next without
/// making the text feel sparse.
const LINE_HEIGHT_MULTIPLIER: f64 = 1.5;

/// RDP simplification tolerance, expressed as a fraction of the stroke's
/// straight-line length. A 6 km stroke gets simplified with ~700 m
/// perpendicular tolerance, which collapses Hershey curves to 3-4 control
/// points while leaving simple straight strokes untouched.
const SIMPLIFY_FRACTION: f64 = 0.12;

/// Floor on the simplification tolerance — for very short strokes (e.g. a
/// dot or a serif segment), don't go below this many metres.
const SIMPLIFY_MIN_M: f64 = 200.0;

/// Extra space inserted between successive glyph cells, expressed as a
/// fraction of cap height. Hershey glyphs are designed to touch at the cell
/// boundary; we want visual separation between letters AND we want enough
/// room that per-letter local jitter (`SEARCH_RADIUS_FRACTION` × letter
/// height) doesn't push a letter into the next letter's column. 0.6 cap
/// heights gives ~half a letter-width of padding between glyphs.
const TRACKING_FACTOR: f64 = 0.6;

/// Per-letter local search radius as a fraction of letter height. Bounded
/// below `TRACKING_FACTOR / 2` so two adjacent letters can't drift into
/// each other's column.
const SEARCH_RADIUS_FRACTION: f64 = 0.25;

/// One stroke's worth of waypoints in (lat, lon).
pub type Stroke = Vec<LatLon>;

/// One glyph as laid out within the message, with its strokes already
/// projected and simplified. Per-letter optimisation jitters `centre_nominal`
/// + bearing and re-layouts; this struct is the "where the message would
/// like this letter to go" anchor.
#[derive(Clone, Debug)]
pub struct GlyphPlacement {
    pub ch: char,
    /// Geometric centre of the glyph cell, projected onto the sphere.
    pub centre_nominal: LatLon,
    /// Glyph cell width along the baseline, metres.
    pub cell_w_m: f64,
    /// Glyph cell height (= letter_height_m for the active font), metres.
    pub cell_h_m: f64,
    /// Already-simplified strokes. In source order; the rideable track
    /// traces them in this order with pen-up connectors between them.
    pub strokes: Vec<Stroke>,
}

#[derive(Clone, Debug)]
pub struct TextLayout {
    /// One entry per non-whitespace, font-known character. Whitespace and
    /// missing glyphs do not produce entries.
    pub glyphs: Vec<GlyphPlacement>,
    /// Bounding box of the laid-out text along/across the baseline, metres.
    pub bbox_along_m: f64,
    pub bbox_across_m: f64,
    /// Number of glyphs that had no font data (e.g. non-ASCII), echoed back
    /// so the FE can warn the user instead of silently dropping them.
    pub missing_glyphs: usize,
}

impl TextLayout {
    /// All strokes flattened, in render order. Convenience for tests and
    /// rendering paths that don't care about per-letter grouping.
    pub fn flat_strokes(&self) -> Vec<Stroke> {
        let mut out = Vec::new();
        for g in &self.glyphs {
            for s in &g.strokes {
                out.push(s.clone());
            }
        }
        out
    }
}

/// Lay out `text` centred on `center`, oriented along `bearing_rad` (CW from
/// north, the geographic convention). Capital-letter height is
/// `letter_height_m`; word-wrap kicks in when a line's horizontal extent
/// would exceed `wrap_m` (pass 0 to disable). Newlines are honoured.
///
/// Each Hershey stroke is RDP-simplified before being returned, so caller
/// code can route legs without worrying about snap noise dominating the
/// stroke's shape.
pub fn layout_text(
    text: &str,
    center: LatLon,
    bearing_rad: f64,
    letter_height_m: f64,
    wrap_m: f64,
    font: Font,
) -> TextLayout {
    let glyphs = font.glyphs();
    let cap_height = font.cap_height();
    let line_height = cap_height * LINE_HEIGHT_MULTIPLIER;
    let scale = letter_height_m / cap_height; // metres per font unit
    let wrap_units = if wrap_m > 0.0 {
        (wrap_m / scale).max(40.0)
    } else {
        f64::INFINITY
    };
    let cos_b = bearing_rad.cos();
    let sin_b = bearing_rad.sin();

    // First pass: place each glyph in font-unit (x_along, line_index) coords,
    // wrapping at word boundaries when the next non-space token would
    // overflow `wrap_units`.
    struct Placed {
        ch: char,
        x: f64,
        line: usize,
    }
    let mut placed: Vec<Placed> = Vec::new();
    let mut missing = 0usize;
    let mut max_x_seen = 0.0_f64;
    let mut line_idx = 0usize;

    for (li, line) in text.split('\n').enumerate() {
        if li > 0 {
            line_idx += 1;
        }
        let mut x = 0.0_f64;
        // Tokenise into runs of whitespace and runs of non-whitespace so we
        // can wrap at word boundaries.
        let mut tokens: Vec<String> = Vec::new();
        let mut cur = String::new();
        let mut cur_is_space = false;
        for ch in line.chars() {
            let is_space = ch == ' ';
            if cur.is_empty() {
                cur.push(ch);
                cur_is_space = is_space;
            } else if is_space == cur_is_space {
                cur.push(ch);
            } else {
                tokens.push(std::mem::take(&mut cur));
                cur.push(ch);
                cur_is_space = is_space;
            }
        }
        if !cur.is_empty() {
            tokens.push(cur);
        }

        let tracking = cap_height * TRACKING_FACTOR;
        for tok in tokens {
            // Token width already accounts for tracking inserted before each
            // non-first character of the token; no tracking is added before
            // the first character of each token because either it's the
            // start of a line (x = 0) or the previous token was whitespace
            // (whose own advance already provides separation).
            let n_chars = tok.chars().count();
            let tok_chars_w: f64 = tok
                .chars()
                .map(|c| glyph_advance(c, glyphs).unwrap_or(0.0))
                .sum();
            let tok_tracking_w = if n_chars > 1 {
                tracking * (n_chars as f64 - 1.0)
            } else {
                0.0
            };
            let tok_width = tok_chars_w + tok_tracking_w;
            let is_space_tok = tok.starts_with(' ');
            if !is_space_tok && x + tok_width > wrap_units && x > 0.0 {
                line_idx += 1;
                x = 0.0;
            }
            for (idx, ch) in tok.chars().enumerate() {
                let adv = match glyph_advance(ch, glyphs) {
                    Some(w) => w,
                    None => {
                        missing += 1;
                        continue;
                    }
                };
                if idx > 0 {
                    // Add tracking only between letters within a token —
                    // word spaces already create a wide gap via the space
                    // glyph's own advance, so we don't extend that further.
                    x += tracking;
                }
                placed.push(Placed {
                    ch,
                    x,
                    line: line_idx,
                });
                x += adv;
                if x > max_x_seen {
                    max_x_seen = x;
                }
            }
        }
    }

    let total_h_units = (line_idx as f64 + 1.0) * line_height;
    let total_w_units = max_x_seen.max(1.0);
    // Block coordinates have origin at block centre. half_w / half_h shift
    // the upper-left-anchored layout so that the geometric centre of the
    // bounding box sits on `center`.
    let half_w = total_w_units * 0.5;
    let half_h = total_h_units * 0.5;

    // Second pass: project each glyph's strokes into LatLon, grouped per
    // glyph so per-letter placement can later replace one glyph's strokes
    // without affecting the others.
    let mut out_glyphs: Vec<GlyphPlacement> = Vec::new();
    let simplify_eps_floor = SIMPLIFY_MIN_M;
    for pg in &placed {
        let glyph = match glyphs.get(pg.ch as usize).and_then(|g| *g) {
            Some(g) => g,
            None => continue,
        };
        if glyph.strokes.is_empty() {
            continue;
        }
        // Hershey glyph data is stored relative to the glyph cell's left
        // margin, but the magnitude varies — anchor on the leftmost x in the
        // strokes so consecutive glyphs touch at the advance boundary instead
        // of leaving an arbitrary gap.
        let mut min_glyph_x: i16 = i16::MAX;
        for st in glyph.strokes {
            for &(x, _) in *st {
                if x < min_glyph_x {
                    min_glyph_x = x;
                }
            }
        }
        let glyph_left = min_glyph_x as f64;

        // Compute the cell centre in block coords and project to LatLon.
        // Cell width = glyph.width font units; cell height = cap_height.
        let cell_cx_block = pg.x + (glyph.width as f64) * 0.5 - half_w;
        let cell_cy_block = half_h - cap_height * 0.5 - pg.line as f64 * line_height;
        let centre_nominal = project_offset(
            center,
            cell_cx_block * scale,
            cell_cy_block * scale,
            sin_b,
            cos_b,
        );

        let mut strokes_g: Vec<Stroke> = Vec::new();
        for st in glyph.strokes {
            if st.len() < 2 {
                continue;
            }
            let mut waypoints: Stroke = Vec::with_capacity(st.len());
            for &(gx, gy) in *st {
                let x_block = pg.x + (gx as f64 - glyph_left) - half_w;
                let y_block = half_h + gy as f64 - pg.line as f64 * line_height;
                let x_along_m = x_block * scale;
                let y_above_m = y_block * scale;
                waypoints.push(project_offset(center, x_along_m, y_above_m, sin_b, cos_b));
            }
            let stroke_len_m = polyline_length_m(&waypoints);
            let eps = (stroke_len_m * SIMPLIFY_FRACTION).max(simplify_eps_floor);
            let simplified = rdp_simplify(&waypoints, eps);
            if simplified.len() >= 2 {
                strokes_g.push(simplified);
            }
        }
        if !strokes_g.is_empty() {
            out_glyphs.push(GlyphPlacement {
                ch: pg.ch,
                centre_nominal,
                cell_w_m: glyph.width as f64 * scale,
                cell_h_m: cap_height * scale,
                strokes: strokes_g,
            });
        }
    }

    TextLayout {
        glyphs: out_glyphs,
        bbox_along_m: total_w_units * scale,
        bbox_across_m: total_h_units * scale,
        missing_glyphs: missing,
    }
}

fn glyph_advance(ch: char, glyphs: &[Option<&'static Glyph>; 128]) -> Option<f64> {
    glyphs
        .get(ch as usize)
        .and_then(|g| *g)
        .map(|g| g.width as f64)
}

/// Convert a (x_along_baseline, y_above_baseline) block-relative offset in
/// metres into geographic LatLon, given the block centre and the precomputed
/// baseline bearing's sin/cos. Positive x is along the writing direction;
/// positive y is "up in text" (perpendicular to writing).
fn project_offset(
    center: LatLon,
    x_along_m: f64,
    y_above_m: f64,
    sin_b: f64,
    cos_b: f64,
) -> LatLon {
    // u_hat (writing dir) = (sin β, cos β) in (east, north)
    // v_hat ("up" in text) = u rotated 90° CCW = (-cos β, sin β)
    let east_m = x_along_m * sin_b - y_above_m * cos_b;
    let north_m = x_along_m * cos_b + y_above_m * sin_b;
    let dist = (east_m * east_m + north_m * north_m).sqrt();
    if dist < 1e-3 {
        return center;
    }
    let bearing = east_m.atan2(north_m); // CW from north
    destination(center, bearing, dist)
}

fn polyline_length_m(pts: &[LatLon]) -> f64 {
    pts.windows(2).map(|w| haversine(w[0], w[1])).sum()
}

/// Ramer-Douglas-Peucker polyline simplification, perpendicular tolerance
/// `eps_m` in metres. Endpoints are always kept. For closed/near-closed
/// strokes RDP reduces to just the endpoints (since both project onto each
/// other through the line a→b), which would erase the loop — we guard
/// against that by dropping the simplified result and keeping the original
/// when collapse-to-nothing happens (see caller).
fn rdp_simplify(pts: &[LatLon], eps_m: f64) -> Vec<LatLon> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    // Special case: closed/near-closed loop. RDP between identical endpoints
    // collapses everything to two points. Detect by endpoint distance vs the
    // total polyline length — if endpoints are within 25 % of total length
    // of each other, treat as closed and split into two halves around the
    // midpoint, simplifying each half independently.
    let total = polyline_length_m(pts);
    let endpoint_gap = haversine(pts[0], pts[pts.len() - 1]);
    if total > 0.0 && endpoint_gap < total * 0.25 && pts.len() >= 4 {
        let mid = pts.len() / 2;
        let mut a = rdp_simplify(&pts[..=mid], eps_m);
        let b = rdp_simplify(&pts[mid..], eps_m);
        a.extend_from_slice(&b[1..]); // drop duplicate junction
        return a;
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    rdp_recurse(pts, 0, pts.len() - 1, eps_m, &mut keep);
    pts.iter()
        .zip(keep.iter())
        .filter_map(|(p, &k)| if k { Some(*p) } else { None })
        .collect()
}

fn rdp_recurse(pts: &[LatLon], lo: usize, hi: usize, eps_m: f64, keep: &mut [bool]) {
    if hi <= lo + 1 {
        return;
    }
    let a = pts[lo];
    let b = pts[hi];
    let mut max_d = 0.0_f64;
    let mut max_i = lo;
    for i in (lo + 1)..hi {
        let d = perpendicular_distance_m(pts[i], a, b);
        if d > max_d {
            max_d = d;
            max_i = i;
        }
    }
    if max_d > eps_m {
        keep[max_i] = true;
        rdp_recurse(pts, lo, max_i, eps_m, keep);
        rdp_recurse(pts, max_i, hi, eps_m, keep);
    }
}

/// Perpendicular distance from `p` to the segment `a-b`, metres. For very
/// short segments (a ≈ b) we fall back to point-to-point haversine.
fn perpendicular_distance_m(p: LatLon, a: LatLon, b: LatLon) -> f64 {
    let ab = haversine(a, b);
    if ab < 1e-3 {
        return haversine(a, p);
    }
    // Treat the (small) span as flat: project p onto the line a→b.
    // East-north offsets relative to `a` for both p and b.
    let to_en = |q: LatLon| -> (f64, f64) {
        let mid_lat = (a.lat + q.lat) * 0.5;
        let east = (q.lon - a.lon).to_radians() * 6_371_008.8 * mid_lat.to_radians().cos();
        let north = (q.lat - a.lat).to_radians() * 6_371_008.8;
        (east, north)
    };
    let (px, py) = to_en(p);
    let (bx, by) = to_en(b);
    let len2 = bx * bx + by * by;
    if len2 < 1e-9 {
        return (px * px + py * py).sqrt();
    }
    let t = ((px * bx + py * by) / len2).clamp(0.0, 1.0);
    let dx = px - t * bx;
    let dy = py - t * by;
    (dx * dx + dy * dy).sqrt()
}

#[derive(Debug, Clone)]
pub struct StrokeRoute {
    /// Concatenated routed points along this stroke (one polyline).
    pub points: Vec<LatLon>,
    pub total_length_m: f64,
    pub edges: Vec<EdgeData>,
    /// Sum of straight-line distances between intended (simplified) waypoints
    /// — the "ideal" stroke length, used for detour gating + scoring.
    pub intended_length_m: f64,
}

/// Route a (simplified) stroke leg-by-leg using the existing A* pipeline.
/// Returns the concatenated polyline. Each leg's corridor reference is its
/// own a→b line, but because the stroke has been pre-simplified to ~2-4
/// points the per-leg corridor still pins the overall stroke shape. Errors
/// if any leg fails to route (snap displacement too large, no path, etc.).
pub fn route_stroke(
    graph: &Graph,
    waypoints: &[LatLon],
    params: &RouteParams,
    snap_tolerance_m: f64,
) -> Result<StrokeRoute, String> {
    if waypoints.len() < 2 {
        return Err("stroke has no legs".into());
    }
    let mut points: Vec<LatLon> = Vec::new();
    let mut edges: Vec<EdgeData> = Vec::new();
    let mut total = 0.0_f64;
    let mut intended = 0.0_f64;
    for (i, w) in waypoints.windows(2).enumerate() {
        let pair = graph
            .closest_connected_pair(w[0], w[1])
            .ok_or_else(|| format!("leg {}: graph empty", i + 1))?;
        let (s_idx, e_idx, sd, ed) = pair;
        if sd > snap_tolerance_m || ed > snap_tolerance_m {
            return Err(format!(
                "leg {}: snap displacement too large ({:.0} m / {:.0} m)",
                i + 1,
                sd,
                ed
            ));
        }
        let leg = route::shortest(graph, s_idx, e_idx, w[0], w[1], params)
            .map_err(|e| format!("leg {}: {}", i + 1, e))?;
        if points.is_empty() {
            points.extend(&leg.points);
        } else {
            // Drop the duplicate junction point shared with the previous leg.
            points.extend(&leg.points[1..]);
        }
        edges.extend(leg.edges.iter().cloned());
        total += leg.total_length_m;
        intended += haversine(w[0], w[1]);
    }
    Ok(StrokeRoute {
        points,
        total_length_m: total,
        edges,
        intended_length_m: intended,
    })
}

/// Overall fidelity score for a routed text result. Lower = better. Detour
/// ratio (routed length vs intended length, summed across all strokes)
/// dominates; missing glyphs add a small fixed penalty so candidates with the
/// same routing fidelity still prefer fewer dropped characters.
pub fn text_score(total_routed_m: f64, total_intended_m: f64, missing_glyphs: usize) -> f64 {
    let detour = if total_intended_m > 0.0 {
        (total_routed_m / total_intended_m - 1.0).max(0.0)
    } else {
        0.0
    };
    100.0 * detour + 0.5 * missing_glyphs as f64
}

/// One letter, after per-letter local optimisation: each of its Hershey
/// strokes routed within `detour_cap` at the chosen centre/bearing.
#[derive(Debug, Clone)]
pub struct LetterRoute {
    pub ch: char,
    /// Where this letter actually ended up after local jitter (the *chosen*
    /// centre, possibly different from the message's nominal centre for this
    /// letter).
    pub centre_chosen: LatLon,
    pub bearing_rad: f64,
    /// One StrokeRoute per Hershey stroke of this letter, in source order.
    pub stroke_routes: Vec<StrokeRoute>,
    pub max_stroke_detour: f64,
    pub avg_stroke_detour: f64,
    pub total_intended_m: f64,
    pub total_routed_m: f64,
}

/// One pen-up connector — the rideable segment that bridges two strokes (or
/// two letters). Drawn thinly on the map so it doesn't compete with the
/// letter strokes for visual attention; included in the GPX so the whole
/// thing is one continuous track.
#[derive(Debug, Clone)]
pub struct PenUp {
    pub points: Vec<LatLon>,
    pub length_m: f64,
}

/// One full message placement. The rideable order is:
/// letter[0].stroke[0] → pen_up_within_letter_0 → letter[0].stroke[1] → …
///   → pen_up_between_letter_0_and_1 → letter[1].stroke[0] → …
/// `pen_ups` and the implied stroke ordering are kept aligned so the FE can
/// emit them in interleaved render/ride order.
#[derive(Debug, Clone)]
pub struct MessagePlacement {
    /// Initial outer-search anchor (the layout's nominal centre/bearing).
    pub anchor_centre: LatLon,
    pub anchor_bearing_rad: f64,
    pub letters: Vec<LetterRoute>,
    /// Pen-ups in ride order. There is one pen-up between every consecutive
    /// pair of strokes across the whole message; `pen_ups.len()` equals
    /// `total_stroke_count - 1`.
    pub pen_ups: Vec<PenUp>,
    pub total_intended_m: f64,
    pub total_routed_m: f64,
    pub total_pen_up_m: f64,
    pub max_stroke_detour: f64,
    pub bbox_along_m: f64,
    pub bbox_across_m: f64,
}

/// Loose-corridor route used for pen-up connectors. Pen-ups don't need
/// shape fidelity — they're just "get from the end of stroke A to the
/// start of stroke B by the cheapest path". A wide corridor + low α lets
/// the router take the natural shortcut.
fn route_pen_up(
    graph: &Graph,
    from: LatLon,
    to: LatLon,
    modes: u8,
    paved_only: bool,
) -> Option<PenUp> {
    let dist = haversine(from, to);
    if dist < 1.0 {
        return Some(PenUp {
            points: vec![from, to],
            length_m: 0.0,
        });
    }
    let pair = graph.closest_connected_pair(from, to)?;
    let (s_idx, e_idx, _, _) = pair;
    // Corridor wide enough that "shortest path" basically wins; α small so
    // we don't bend the connector toward the straight line and waste km.
    let half = (dist * 1.5).max(2_000.0);
    let pen_params = RouteParams {
        modes,
        half_width_m: half,
        alpha: 0.5,
        corridor_max_m: half * 3.0,
        paved_only,
    };
    let leg = route::shortest(graph, s_idx, e_idx, from, to, &pen_params).ok()?;
    Some(PenUp {
        length_m: leg.total_length_m,
        points: leg.points,
    })
}

/// Search for the best local placement of a single letter. We sample
/// `n_candidates` (centre, bearing) jitter offsets around the nominal
/// position, route every stroke of the glyph at each, and keep the one with
/// the lowest max-stroke-detour. Returns None if no candidate has every
/// stroke within the detour cap — that means the message-level search needs
/// to retry with a different anchor.
#[allow(clippy::too_many_arguments)]
fn search_letter_placement(
    graph: &Graph,
    glyph_nominal: &GlyphPlacement,
    nominal_bearing_rad: f64,
    letter_height_m: f64,
    font: Font,
    params: &RouteParams,
    snap_tol: f64,
    detour_cap: f64,
    search_radius_m: f64,
    bearing_jitter_rad: f64,
    n_candidates: usize,
    seed: u64,
) -> Option<LetterRoute> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    let ch = glyph_nominal.ch;
    let nominal_centre = glyph_nominal.centre_nominal;
    let mut rng = StdRng::seed_from_u64(seed);
    // Always include the no-jitter (0, 0, 0) candidate first — for areas
    // where the nominal placement already works, skip the random sampling.
    let mut tried: Vec<(LatLon, f64)> = Vec::with_capacity(n_candidates + 1);
    tried.push((nominal_centre, nominal_bearing_rad));
    for _ in 0..n_candidates {
        let r = rng.gen_range(0.0..search_radius_m);
        let theta = rng.gen_range(0.0..(2.0 * std::f64::consts::PI));
        let east_m = r * theta.cos();
        let north_m = r * theta.sin();
        let dist = (east_m * east_m + north_m * north_m).sqrt();
        let bearing_off = if dist < 1e-3 {
            0.0
        } else {
            east_m.atan2(north_m)
        };
        let centre = if dist < 1e-3 {
            nominal_centre
        } else {
            destination(nominal_centre, bearing_off, dist)
        };
        let dtheta = rng.gen_range(-bearing_jitter_rad..bearing_jitter_rad);
        tried.push((centre, nominal_bearing_rad + dtheta));
    }

    let mut best: Option<LetterRoute> = None;
    for (centre, bearing) in tried {
        // Layout this single character at the candidate centre/bearing. The
        // glyph table data is identical to what the message-level layout
        // produced; this re-runs the projection at the new anchor.
        let s = ch.to_string();
        let lay = layout_text(&s, centre, bearing, letter_height_m, 0.0, font);
        if lay.glyphs.is_empty() {
            continue;
        }
        let strokes = &lay.glyphs[0].strokes;
        if strokes.is_empty() {
            continue;
        }
        let mut stroke_routes: Vec<StrokeRoute> = Vec::with_capacity(strokes.len());
        let mut sum_routed = 0.0_f64;
        let mut sum_intended = 0.0_f64;
        let mut max_ratio = 0.0_f64;
        let mut sum_ratio = 0.0_f64;
        let mut ok = true;
        for st in strokes {
            match route_stroke(graph, st, params, snap_tol) {
                Ok(r) => {
                    let intended = r.intended_length_m.max(1.0);
                    let ratio = r.total_length_m / intended;
                    if ratio > detour_cap {
                        ok = false;
                        break;
                    }
                    sum_routed += r.total_length_m;
                    sum_intended += r.intended_length_m;
                    sum_ratio += ratio;
                    if ratio > max_ratio {
                        max_ratio = ratio;
                    }
                    stroke_routes.push(r);
                }
                Err(_) => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            continue;
        }
        let candidate = LetterRoute {
            ch,
            centre_chosen: centre,
            bearing_rad: bearing,
            stroke_routes,
            max_stroke_detour: max_ratio,
            avg_stroke_detour: if !strokes.is_empty() {
                sum_ratio / strokes.len() as f64
            } else {
                0.0
            },
            total_intended_m: sum_intended,
            total_routed_m: sum_routed,
        };
        match &best {
            None => best = Some(candidate),
            Some(b) if candidate.max_stroke_detour < b.max_stroke_detour => best = Some(candidate),
            _ => {}
        }
        // Early-out: if the no-jitter placement is already excellent
        // (every stroke under 1.2× intended), don't bother sampling more.
        if let Some(b) = &best {
            if b.max_stroke_detour < 1.2 {
                return Some(b.clone());
            }
        }
    }
    best
}

/// Greedy per-letter placement: for each letter in turn, locally optimise
/// its centre/bearing (around the nominal position predicted from the
/// previous letter's actual placement); then route pen-up connectors
/// between consecutive strokes. Returns None if any letter's local search
/// can't fit the detour cap, or if any pen-up fails to route.
#[allow(clippy::too_many_arguments)]
fn place_message_greedy(
    graph: &Graph,
    text: &str,
    anchor_centre: LatLon,
    anchor_bearing_rad: f64,
    letter_height_m: f64,
    wrap_m: f64,
    font: Font,
    params: &RouteParams,
    snap_tol: f64,
    detour_cap: f64,
    n_local_candidates: usize,
    seed: u64,
) -> Option<MessagePlacement> {
    let layout = layout_text(
        text,
        anchor_centre,
        anchor_bearing_rad,
        letter_height_m,
        wrap_m,
        font,
    );
    if layout.glyphs.is_empty() {
        return None;
    }

    // Local search radius: bounded BELOW the inter-letter gap so two
    // adjacent letters can't drift into each other's column. Inter-letter
    // gap = `TRACKING_FACTOR × cap_height × scale = TRACKING_FACTOR ×
    // letter_height`; we use half of that.
    let search_radius_m = letter_height_m * SEARCH_RADIUS_FRACTION;
    let bearing_jitter_rad = 10.0_f64.to_radians();

    let mut letters: Vec<LetterRoute> = Vec::with_capacity(layout.glyphs.len());
    // Translation accumulated from prior letters' chosen centres vs their
    // nominal centres. We use it to predict the nominal position for the
    // next letter — if letter N drifted 2 km north of nominal, letter N+1
    // probably wants to start its search 2 km north of its nominal too.
    let mut accumulated_offset_east = 0.0_f64;
    let mut accumulated_offset_north = 0.0_f64;

    for (i, glyph_nom) in layout.glyphs.iter().enumerate() {
        // Apply the accumulated offset to the nominal centre.
        let nominal = if i == 0
            || (accumulated_offset_east.abs() < 1.0 && accumulated_offset_north.abs() < 1.0)
        {
            glyph_nom.clone()
        } else {
            let dist = (accumulated_offset_east * accumulated_offset_east
                + accumulated_offset_north * accumulated_offset_north)
                .sqrt();
            let bearing = accumulated_offset_east.atan2(accumulated_offset_north);
            let shifted = destination(glyph_nom.centre_nominal, bearing, dist);
            GlyphPlacement {
                ch: glyph_nom.ch,
                centre_nominal: shifted,
                cell_w_m: glyph_nom.cell_w_m,
                cell_h_m: glyph_nom.cell_h_m,
                strokes: glyph_nom.strokes.clone(),
            }
        };
        let letter_seed = seed.wrapping_add((i as u64).wrapping_mul(0x9E3779B97F4A7C15));
        let lr = search_letter_placement(
            graph,
            &nominal,
            anchor_bearing_rad,
            letter_height_m,
            font,
            params,
            snap_tol,
            detour_cap,
            search_radius_m,
            bearing_jitter_rad,
            n_local_candidates,
            letter_seed,
        )?;
        // Update the accumulated offset based on actual vs nominal for THIS
        // letter (using the original glyph_nom, not the shifted one — we
        // only carry one letter's worth of drift forward to avoid runaway).
        let actual = lr.centre_chosen;
        let nom = glyph_nom.centre_nominal;
        accumulated_offset_east = (actual.lon - nom.lon).to_radians()
            * 6_371_008.8
            * ((actual.lat + nom.lat) * 0.5).to_radians().cos();
        accumulated_offset_north = (actual.lat - nom.lat).to_radians() * 6_371_008.8;
        letters.push(lr);
    }

    // Route pen-up connectors between consecutive strokes (across letter
    // boundaries and within each letter).
    let mut pen_ups: Vec<PenUp> = Vec::new();
    let mut total_pen_up_m = 0.0_f64;
    // Flatten: walk all (letter_index, stroke_index) tuples in ride order.
    let mut prev_end: Option<LatLon> = None;
    for letter in &letters {
        for sr in &letter.stroke_routes {
            let stroke_start = *sr.points.first().expect("stroke has points");
            let stroke_end = *sr.points.last().expect("stroke has points");
            if let Some(prev) = prev_end {
                let pu = route_pen_up(graph, prev, stroke_start, params.modes, params.paved_only)?;
                total_pen_up_m += pu.length_m;
                pen_ups.push(pu);
            }
            prev_end = Some(stroke_end);
        }
    }

    let total_intended_m: f64 = letters.iter().map(|l| l.total_intended_m).sum();
    let total_routed_m: f64 = letters.iter().map(|l| l.total_routed_m).sum();
    let max_stroke_detour = letters
        .iter()
        .map(|l| l.max_stroke_detour)
        .fold(0.0_f64, f64::max);

    Some(MessagePlacement {
        anchor_centre,
        anchor_bearing_rad,
        letters,
        pen_ups,
        total_intended_m,
        total_routed_m,
        total_pen_up_m,
        max_stroke_detour,
        bbox_along_m: layout.bbox_along_m,
        bbox_across_m: layout.bbox_across_m,
    })
}

/// Outer search across (centre, bearing) anchor points; for each anchor,
/// run the greedy per-letter placement. Returns successful placements
/// ranked by total detour. Per-letter local search makes individual letters
/// nearly always succeed — so the outer search converges fast: usually the
/// first few anchors that land in viable terrain give a result.
#[allow(clippy::too_many_arguments)]
pub fn search_text_placements(
    graph: &Graph,
    text: &str,
    letter_height_m: f64,
    wrap_m: f64,
    font: Font,
    params: &RouteParams,
    bbox: BBox,
    detour_cap: f64,
    n_candidates: usize,
    seed: u64,
    progress: Option<&std::sync::Arc<std::sync::atomic::AtomicUsize>>,
    _no_go: &[NoGoZone],
) -> Vec<MessagePlacement> {
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    use rayon::prelude::*;
    use std::sync::atomic::Ordering;

    // Probe layout to read out the message's bounding box; used to shrink
    // the sampling bbox so the whole block stays inside the loaded graph.
    let probe = layout_text(
        text,
        LatLon::new(0.0, 0.0),
        std::f64::consts::FRAC_PI_2,
        letter_height_m,
        wrap_m,
        font,
    );
    if probe.glyphs.is_empty() {
        return Vec::new();
    }
    // Per-letter placement means we don't need to keep the *whole* message
    // inside the loaded graph at sample time — a candidate where a few
    // edge letters fall outside the graph just rejects naturally when
    // those letters fail to route. Pad by ~1.5 letter heights instead of
    // half the message extent so long messages still have plenty of
    // anchor positions to sample.
    let edge_pad_m = letter_height_m * 1.5;
    let lat_pad = edge_pad_m / 111_000.0;
    let mid_lat = (bbox.lat_min + bbox.lat_max) * 0.5;
    let lon_pad = lat_pad / mid_lat.to_radians().cos().max(1e-3);
    let inner_lat_min = bbox.lat_min + lat_pad;
    let inner_lat_max = bbox.lat_max - lat_pad;
    let inner_lon_min = bbox.lon_min + lon_pad;
    let inner_lon_max = bbox.lon_max - lon_pad;
    if inner_lat_max <= inner_lat_min || inner_lon_max <= inner_lon_min {
        return Vec::new();
    }

    let bearings_deg: [f64; 5] = [60.0, 75.0, 90.0, 105.0, 120.0];
    let n_pos = n_candidates.div_ceil(bearings_deg.len()).max(1);

    let mut rng = StdRng::seed_from_u64(seed);
    let mut anchors: Vec<(LatLon, f64, u64)> = Vec::with_capacity(n_pos * bearings_deg.len());
    for i in 0..n_pos {
        let lat = rng.gen_range(inner_lat_min..inner_lat_max);
        let lon = rng.gen_range(inner_lon_min..inner_lon_max);
        let c = LatLon::new(lat, lon);
        for (j, &b_deg) in bearings_deg.iter().enumerate() {
            let anchor_seed = seed
                .wrapping_add((i as u64).wrapping_mul(0xA5A5A5A5))
                .wrapping_add((j as u64).wrapping_mul(0x5A5A5A5A));
            anchors.push((c, b_deg.to_radians(), anchor_seed));
        }
    }

    let stroke_half_m = params.half_width_m.max(1.0);
    let snap_tol = (stroke_half_m * 2.0).max(2_000.0);
    // Per-letter local candidates. Bigger = more chance any single letter
    // succeeds, but linear cost. 12 is a balance between thoroughness and
    // outer-search throughput.
    let n_local = 12usize;

    let placements: Vec<MessagePlacement> = anchors
        .par_iter()
        .filter_map(|&(centre, bearing, anchor_seed)| {
            let result = place_message_greedy(
                graph,
                text,
                centre,
                bearing,
                letter_height_m,
                wrap_m,
                font,
                params,
                snap_tol,
                detour_cap,
                n_local,
                anchor_seed,
            );
            if let Some(p) = progress {
                p.fetch_add(1, Ordering::Relaxed);
            }
            result
        })
        .collect();

    let mut placements = placements;
    placements.sort_by(|a, b| {
        let ka = a.total_routed_m / a.total_intended_m.max(1.0);
        let kb = b.total_routed_m / b.total_intended_m.max(1.0);
        ka.partial_cmp(&kb).unwrap_or(std::cmp::Ordering::Equal)
    });
    placements
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::haversine;
    use std::f64::consts::FRAC_PI_2;

    fn deg(a: f64, b: f64) -> LatLon {
        LatLon::new(a, b)
    }

    #[test]
    fn rdp_collapses_curve_to_few_points() {
        // 13-point arc spanning ~10 km, sampled finely. With eps = 1.2 km
        // (=12 % of length) the simplification should keep the endpoints and
        // a couple of mid-arc points.
        let center = deg(46.8, 8.2);
        let r = 5_000.0;
        let pts: Vec<LatLon> = (0..=12)
            .map(|i| {
                let theta = std::f64::consts::PI * (i as f64) / 12.0;
                destination(center, theta, r)
            })
            .collect();
        let total = polyline_length_m(&pts);
        let simplified = rdp_simplify(&pts, total * SIMPLIFY_FRACTION);
        assert!(
            simplified.len() <= 5 && simplified.len() >= 2,
            "expected 2..=5 points after RDP, got {}",
            simplified.len()
        );
        assert!(haversine(simplified[0], pts[0]) < 1.0);
        assert!(haversine(*simplified.last().unwrap(), *pts.last().unwrap()) < 1.0);
    }

    #[test]
    fn rdp_keeps_loop_topology() {
        // Closed-ish polygon: 16-point circle. Without the closed-loop guard
        // RDP would collapse the loop to a single line. We expect at least
        // 4 points back so the loop is still recognisably a loop.
        let center = deg(46.8, 8.2);
        let r = 4_000.0;
        let pts: Vec<LatLon> = (0..=16)
            .map(|i| {
                let theta = 2.0 * std::f64::consts::PI * (i as f64) / 16.0;
                destination(center, theta, r)
            })
            .collect();
        let total = polyline_length_m(&pts);
        let simplified = rdp_simplify(&pts, total * SIMPLIFY_FRACTION);
        assert!(
            simplified.len() >= 4,
            "loop collapsed to {} points; should keep ≥ 4",
            simplified.len()
        );
    }

    #[test]
    fn layout_centred_on_supplied_centre() {
        // For 'I' on a horizontal baseline (bearing = π/2 = east), the layout
        // bbox must be centred on the supplied centre — i.e. the average lon
        // of the routed strokes should be very close to centre.lon.
        let centre = deg(46.8, 8.2);
        let lay = layout_text("I", centre, FRAC_PI_2, 10_000.0, 0.0, Font::Simplex);
        assert!(!lay.flat_strokes().is_empty());
        let mut sum_lon = 0.0;
        let mut n = 0;
        for s in &lay.flat_strokes() {
            for p in s {
                sum_lon += p.lon;
                n += 1;
            }
        }
        let mean_lon = sum_lon / n as f64;
        // Single 'I' is a thin vertical bar; its mean lon should be very near
        // centre.lon (within a fraction of letter-height in metres-as-degrees).
        assert!(
            (mean_lon - centre.lon).abs() < 0.05,
            "stroke mean lon {} too far from centre.lon {}",
            mean_lon,
            centre.lon
        );
    }

    #[test]
    fn layout_rotates_with_bearing() {
        // North-bound writing (bearing = 0). "Up in text" is now WEST in
        // geographic terms (90° CCW of writing dir), so the letter top should
        // sit west of centre.
        let centre = deg(46.8, 8.2);
        let lay = layout_text("I", centre, 0.0, 10_000.0, 0.0, Font::Simplex);
        let stroke = &lay.flat_strokes()[0];
        let top = stroke
            .iter()
            .min_by(|p, q| p.lon.partial_cmp(&q.lon).unwrap())
            .unwrap();
        assert!(
            top.lon < centre.lon,
            "letter top lon {} should be west of centre lon {}",
            top.lon,
            centre.lon
        );
    }

    #[test]
    fn layout_wraps_long_text() {
        let centre = deg(46.8, 8.2);
        let lay = layout_text(
            "AAAA AAAA AAAA AAAA",
            centre,
            FRAC_PI_2,
            5_000.0,
            20_000.0,
            Font::Simplex,
        );
        assert!(
            lay.bbox_across_m > 5_000.0 * LINE_HEIGHT_MULTIPLIER,
            "expected multi-line layout, got across={}",
            lay.bbox_across_m
        );
    }

    #[test]
    fn missing_glyph_counted() {
        let centre = deg(46.8, 8.2);
        // 'ñ' is outside the printable-ASCII subset baked into the font.
        let lay = layout_text("Añ", centre, FRAC_PI_2, 5_000.0, 0.0, Font::Simplex);
        assert_eq!(lay.missing_glyphs, 1);
    }

    #[test]
    fn cursive_layout_works() {
        let centre = deg(46.8, 8.2);
        let lay = layout_text("Hi", centre, FRAC_PI_2, 5_000.0, 0.0, Font::Cursive);
        assert!(
            !lay.flat_strokes().is_empty(),
            "cursive should produce strokes"
        );
    }

    #[test]
    fn strokes_are_simplified() {
        let centre = deg(46.8, 8.2);
        let lay = layout_text("O", centre, FRAC_PI_2, 10_000.0, 0.0, Font::Simplex);
        for s in &lay.flat_strokes() {
            assert!(
                s.len() <= 8,
                "expected RDP to leave ≤ 8 points per stroke, got {}",
                s.len()
            );
        }
    }
}
