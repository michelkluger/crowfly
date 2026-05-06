"""Extract Switzerland's outer border from a world GeoJSON, simplify with
Douglas-Peucker, and emit a Rust array literal. One-shot helper — not
part of the build. Run once, paste output into shape.rs."""

import json
import math
import sys
from pathlib import Path

SRC = Path(r"C:\Users\miche\AppData\Local\Temp\che.geojson")
TARGET_POINTS = 100
TOL_HIGH = 0.05  # degrees, search bounds for binary search
TOL_LOW = 0.0001


def find_ch(world):
    for feat in world["features"]:
        props = feat.get("properties", {})
        for key in ("ISO_A3", "ISO3", "iso_a3", "ADM0_A3", "id"):
            if props.get(key) == "CHE":
                return feat
        if props.get("ADMIN") == "Switzerland" or props.get("name") == "Switzerland":
            return feat
    return None


def outer_ring(geom):
    """Return the outer ring of the largest polygon (lon, lat) tuples."""
    if geom["type"] == "Polygon":
        polys = [geom["coordinates"]]
    elif geom["type"] == "MultiPolygon":
        polys = geom["coordinates"]
    else:
        raise SystemExit(f"unexpected geometry: {geom['type']}")
    # Largest = highest vertex count on outer ring (good enough for CH).
    best = max(polys, key=lambda p: len(p[0]))
    return best[0]


def perp_dist(p, a, b):
    """Perpendicular distance from p to segment ab in lon/lat-as-flat space."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx = ax + t * dx
    cy = ay + t * dy
    return math.hypot(px - cx, py - cy)


def douglas_peucker(points, tol):
    """Iterative Douglas-Peucker. points is a list of (lon, lat). Keeps endpoints."""
    if len(points) < 3:
        return list(points)
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        max_d = 0.0
        max_k = -1
        a, b = points[i], points[j]
        for k in range(i + 1, j):
            d = perp_dist(points[k], a, b)
            if d > max_d:
                max_d = d
                max_k = k
        if max_d > tol and max_k != -1:
            keep[max_k] = True
            stack.append((i, max_k))
            stack.append((max_k, j))
    return [p for p, k in zip(points, keep) if k]


def main():
    world = json.loads(SRC.read_text(encoding="utf-8"))
    feat = find_ch(world)
    if feat is None:
        sys.exit("Switzerland not found in source GeoJSON")
    print(f"Found feature: {feat['properties'].get('ADMIN', '?')}", file=sys.stderr)
    ring = outer_ring(feat["geometry"])
    print(f"Raw outer ring: {len(ring)} vertices", file=sys.stderr)

    # Binary search a tolerance that yields ~TARGET_POINTS.
    lo, hi = TOL_LOW, TOL_HIGH
    best = ring
    for _ in range(40):
        mid = (lo + hi) / 2.0
        simp = douglas_peucker(ring, mid)
        if len(simp) > TARGET_POINTS:
            lo = mid
        else:
            hi = mid
            best = simp
        if abs(len(simp) - TARGET_POINTS) <= 2:
            best = simp
            break
    print(f"Simplified to {len(best)} vertices (tol={mid:.5f}°)", file=sys.stderr)

    # GeoJSON outer rings are CCW (right-hand rule). place_shape is agnostic to
    # direction, but reverse to CW so it walks the border N → E → S → W to
    # match the existing convention in shape.rs.
    cw = list(reversed(best))

    # Emit Rust literal — (lat, lon) order to match the rest of shape.rs.
    print("    const POINTS: &[(f64, f64)] = &[")
    for lon, lat in cw:
        print(f"        ({lat:.4f}, {lon:.4f}),")
    print("    ];")


if __name__ == "__main__":
    main()
