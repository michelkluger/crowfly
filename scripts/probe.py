#!/usr/bin/env python3
"""
Probe driver for crowfly's bike routing.

Fires a small panel of A->B requests at a running `crowfly serve` and computes
a battery of geometric quality metrics on each returned route.  The point is
to put numbers on what "good" vs "bad" looks like, so we can iterate the
cost function with measurement instead of vibes.

Usage:
    python scripts/probe.py --label baseline
    python scripts/probe.py --label compressed --compare baseline
"""

from __future__ import annotations
import argparse
import json
import math
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = ROOT / "experiments"
RUNS_DIR.mkdir(exist_ok=True)

# --- Test panel --------------------------------------------------------------

# Fixed A->B pairs picked to exercise different cycling regimes inside the
# Switzerland bbox.  Each is named so the report stays readable.
PANEL: list[dict[str, Any]] = [
    {
        "name": "zh_winti",   # urban-suburban, mostly flat
        "start": [47.3769, 8.5417],
        "end":   [47.4995, 8.7240],
        "width_m": 6000,
    },
    {
        "name": "bn_fr",      # Bern -> Fribourg, rolling
        "start": [46.9481, 7.4474],
        "end":   [46.8060, 7.1614],
        "width_m": 8000,
    },
    {
        "name": "ls_vv",      # Lausanne -> Vevey, lakeside
        "start": [46.5197, 6.6323],
        "end":   [46.4628, 6.8419],
        "width_m": 5000,
    },
    {
        "name": "zh_lu",      # Zurich -> Luzern, hilly
        "start": [47.3769, 8.5417],
        "end":   [47.0502, 8.3093],
        "width_m": 8000,
    },
    {
        "name": "bn_thn",     # Bern -> Thun, valley
        "start": [46.9481, 7.4474],
        "end":   [46.7494, 7.6280],
        "width_m": 6000,
    },
]

MODES = "foot,bike"
ALTERNATIVES = 1
ELEV_SAMPLES = 80

# --- Geometry ---------------------------------------------------------------

def haversine_m(p, q) -> float:
    """Distance between [lon,lat] pairs in metres."""
    lon1, lat1 = math.radians(p[0]), math.radians(p[1])
    lon2, lat2 = math.radians(q[0]), math.radians(q[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2 * 6_371_000 * math.asin(math.sqrt(a))

def bearing_deg(p, q) -> float:
    """Initial bearing from p to q, [lon,lat] pairs, degrees."""
    lon1, lat1 = math.radians(p[0]), math.radians(p[1])
    lon2, lat2 = math.radians(q[0]), math.radians(q[1])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

def angular_diff(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)

# --- Metrics ----------------------------------------------------------------

@dataclass
class Metrics:
    name: str
    ref_km: float
    real_km: float
    detour_pct: float
    max_dev_m: float
    ascent_m: float | None
    ascent_per_km: float | None
    # Geometric quality
    turn_density_dps_per_km: float   # mean |Δbearing| per km, angle in degrees
    sharp_turns_per_km: float        # count of turns >= 60 deg per km
    hairpins_per_km: float           # turns >= 110 deg per km
    detour_loops: int                # self-proximity events (real loops)
    surface_transitions_per_km: float

def coalesce_close_points(coords: list[list[float]], min_step_m: float = 12.0) -> list[list[float]]:
    """Drop near-coincident consecutive points so bearing computations aren't
    dominated by GPS noise / OSM micro-segments.  Keeps the first point and
    every subsequent point that is at least `min_step_m` from the kept tail."""
    if not coords:
        return coords
    out = [coords[0]]
    for p in coords[1:]:
        if haversine_m(out[-1], p) >= min_step_m:
            out.append(p)
    if out[-1] != coords[-1]:
        out.append(coords[-1])
    return out

def turn_metrics(coords: list[list[float]]) -> tuple[float, float, float]:
    """Returns (mean_dps_per_km, sharp_per_km, hairpin_per_km)."""
    if len(coords) < 3:
        return 0.0, 0.0, 0.0
    coalesced = coalesce_close_points(coords, min_step_m=15.0)
    total_deg = 0.0
    total_m = 0.0
    sharp = 0
    hairpin = 0
    for i in range(1, len(coalesced) - 1):
        a, b, c = coalesced[i-1], coalesced[i], coalesced[i+1]
        seg_m = haversine_m(a, b)
        total_m += seg_m
        if seg_m < 5.0:
            continue
        d = angular_diff(bearing_deg(a, b), bearing_deg(b, c))
        total_deg += d
        if d >= 60.0:
            sharp += 1
        if d >= 110.0:
            hairpin += 1
    km = max(total_m / 1000.0, 1e-6)
    return total_deg / km, sharp / km, hairpin / km

def detour_loop_count(coords: list[list[float]]) -> int:
    """Counts self-proximity events: pairs (i, j) where the polyline returns
    close to its earlier path despite having walked a meaningful distance.
    Subsamples aggressively to stay O(n^2 / k^2)."""
    if len(coords) < 200:
        return 0
    stride = max(1, len(coords) // 600)  # cap pairs at ~360k
    sampled = coords[::stride]
    # Cumulative along-line distance for sampled indices
    cum = [0.0]
    for i in range(1, len(sampled)):
        cum.append(cum[-1] + haversine_m(sampled[i-1], sampled[i]))
    loops = 0
    used = [False] * len(sampled)
    for i in range(len(sampled)):
        if used[i]:
            continue
        for j in range(i + 30, len(sampled)):
            if used[j]:
                continue
            d = haversine_m(sampled[i], sampled[j])
            if d > 60.0:
                continue
            along = cum[j] - cum[i]
            if along > 1500.0:
                loops += 1
                # Mask out a window so one big detour doesn't double-count.
                for k in range(max(0, i - 2), min(len(sampled), j + 3)):
                    used[k] = True
                break
    return loops

def surface_transitions(surface: list[dict[str, Any]], real_km: float) -> float:
    """We don't get a per-segment surface stream from the API — only totals.
    As a proxy, the *number of surface labels present* with non-trivial share
    correlates with fragmentation: a clean route uses 1-2 surfaces, a
    fragmented one uses 4-5.  Returns transitions-per-km estimate.  Imperfect
    but stable across runs and useful as a comparator."""
    significant = sum(1 for s in surface if s.get("km", 0) >= 0.3)
    # Heuristic: each significant surface beyond the first implies a handful
    # of transitions per km of route.
    return max(0, significant - 1) / max(real_km, 1.0)

def metrics_from_result(name: str, r: dict[str, Any]) -> Metrics:
    coords = r["coords"]
    ref_km = r["ref_km"]
    real_km = r["real_km"]
    detour_pct = 100.0 * (real_km / ref_km - 1.0)
    elev = r.get("elevation")
    ascent = elev["ascent_m"] if elev else None
    apk = (ascent / real_km) if (ascent is not None and real_km > 0) else None
    td, sh, hp = turn_metrics(coords)
    loops = detour_loop_count(coords)
    surf = surface_transitions(r.get("surface", []), real_km)
    return Metrics(
        name=name,
        ref_km=ref_km,
        real_km=real_km,
        detour_pct=detour_pct,
        max_dev_m=r["max_dev_m"],
        ascent_m=ascent,
        ascent_per_km=apk,
        turn_density_dps_per_km=td,
        sharp_turns_per_km=sh,
        hairpins_per_km=hp,
        detour_loops=loops,
        surface_transitions_per_km=surf,
    )

# --- HTTP driver ------------------------------------------------------------

def post(host: str, path: str, body: dict[str, Any], timeout: int = 120) -> dict[str, Any]:
    url = host.rstrip("/") + path
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))

def run_panel(host: str, label: str) -> dict[str, Any]:
    out: dict[str, Any] = {"label": label, "host": host, "results": []}
    for tc in PANEL:
        body = {
            "start": tc["start"],
            "end":   tc["end"],
            "modes": MODES,
            "width_m": tc["width_m"],
            "alternatives": ALTERNATIVES,
            "elevation_samples": ELEV_SAMPLES,
        }
        t0 = time.time()
        try:
            resp = post(host, "/api/between", body)
        except urllib.error.HTTPError as e:
            print(f"  {tc['name']}: HTTP {e.code} {e.read().decode('utf-8','ignore')[:200]}")
            continue
        except Exception as e:
            print(f"  {tc['name']}: error {e}")
            continue
        elapsed = time.time() - t0
        results = resp.get("results", [])
        if not results:
            print(f"  {tc['name']}: no results")
            continue
        m = metrics_from_result(tc["name"], results[0])
        out["results"].append(asdict(m))
        print(f"  {tc['name']:>10}: {m.real_km:6.1f}km det+{m.detour_pct:5.1f}%  "
              f"turns {m.turn_density_dps_per_km:6.1f}°/km  "
              f"sharp {m.sharp_turns_per_km:5.2f}  hp {m.hairpins_per_km:4.2f}  "
              f"loops {m.detour_loops:>2}  surf {m.surface_transitions_per_km:4.2f}  "
              f"asc {m.ascent_m or 0:.0f}m  ({elapsed:.1f}s)")
    return out

# --- Reporting --------------------------------------------------------------

def aggregate(run: dict[str, Any]) -> dict[str, float]:
    rs = run.get("results", [])
    if not rs:
        return {}
    def mean(key: str) -> float:
        vals = [r[key] for r in rs if r.get(key) is not None]
        return statistics.fmean(vals) if vals else 0.0
    return {
        "panel_size": len(rs),
        "mean_detour_pct":          mean("detour_pct"),
        "mean_turn_density":        mean("turn_density_dps_per_km"),
        "mean_sharp_per_km":        mean("sharp_turns_per_km"),
        "mean_hairpins_per_km":     mean("hairpins_per_km"),
        "mean_detour_loops":        mean("detour_loops"),
        "mean_surf_transitions":    mean("surface_transitions_per_km"),
        "mean_ascent_per_km":       mean("ascent_per_km"),
        "mean_max_dev_m":           mean("max_dev_m"),
    }

def print_summary(run: dict[str, Any], compare: dict[str, Any] | None = None) -> None:
    agg = aggregate(run)
    if not agg:
        print("(no results to summarise)")
        return
    print(f"\n=== summary [{run['label']}] ===")
    cmp_agg = aggregate(compare) if compare else None
    rows = [
        ("detour %",            "mean_detour_pct",         "{:6.2f}"),
        ("turn density °/km",   "mean_turn_density",       "{:6.1f}"),
        ("sharp turns /km",     "mean_sharp_per_km",       "{:6.2f}"),
        ("hairpins /km",        "mean_hairpins_per_km",    "{:6.2f}"),
        ("detour loops",        "mean_detour_loops",       "{:6.2f}"),
        ("surf transitions /km","mean_surf_transitions",   "{:6.2f}"),
        ("ascent m/km",         "mean_ascent_per_km",      "{:6.1f}"),
        ("max dev m",           "mean_max_dev_m",          "{:6.0f}"),
    ]
    for label, key, fmt in rows:
        cur = agg.get(key, 0.0)
        if cmp_agg:
            base = cmp_agg.get(key, 0.0)
            delta = cur - base
            arrow = "↓" if delta < 0 else ("↑" if delta > 0 else "·")
            print(f"  {label:<24} {fmt.format(cur)}  vs {fmt.format(base)}  {arrow}{abs(delta):.2f}")
        else:
            print(f"  {label:<24} {fmt.format(cur)}")

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="http://127.0.0.1:7878")
    ap.add_argument("--label", required=True, help="run label; saved as experiments/<label>.json")
    ap.add_argument("--compare", default=None, help="optional prior label to diff against")
    args = ap.parse_args()

    print(f"=== probe [{args.label}] -> {args.host} ===")
    run = run_panel(args.host, args.label)
    out_path = RUNS_DIR / f"{args.label}.json"
    out_path.write_text(json.dumps(run, indent=2))
    print(f"wrote {out_path}")
    cmp = None
    if args.compare:
        cmp_path = RUNS_DIR / f"{args.compare}.json"
        if cmp_path.exists():
            cmp = json.loads(cmp_path.read_text())
        else:
            print(f"(compare file {cmp_path} not found, skipping diff)")
    print_summary(run, cmp)
    return 0

if __name__ == "__main__":
    sys.exit(main())
