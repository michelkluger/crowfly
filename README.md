# crowfly

<img width="1769" height="1008" alt="image" src="https://github.com/user-attachments/assets/9a8bbb5c-553c-4cb7-9af8-cb7262302c1f" />



Plan and validate "as-the-crow-flies" crossings of a country — the
[Project Azimut Nord](https://www.strava.com/activities/14070438269) idea
(start at point A, end at point B, stay inside a 5–10 km corridor the whole
way), with three ways to use it:

- **Interactive web app** (`crowfly serve`) — local server with a clickable
  map and an "auto-research" button that returns many ranked suggestions.
- **`explore` CLI** — fan out from a fixed start over a sweep of bearings and
  distances, rank the survivors, write a static HTML leaderboard.
- **`plan` CLI** — score one specific line A → B and emit GPX + HTML viewer.

Suggestions come with: surface breakdown (paved / gravel / path / unpaved /
ferry), elevation profile (ascent, descent, max, min, sparkline), corridor
deviation, ferry segments, and a one-click GPX download. The map basemap
toggles between street (CartoDB Voyager) and terrain (OpenTopoMap).

## Quick start — interactive

```bash
cargo build --release

# First run downloads ~500 MB Switzerland PBF from Geofabrik into ./osm-cache/
./target/release/crowfly serve --region switzerland --modes foot,bike,mtb
#   ⇨  Open  http://127.0.0.1:7878/  in your browser
```

In the browser:

1. Click the map to drop **A** (blue), click again for **B** (red), then hit
   **"Find routes between A and B"** — you get up to 3 alternatives that vary
   in corridor strictness.
2. Or just hit **"Suggest routes across the country"** — server samples
   diverse start/end pairs nationwide, scores them, returns the easiest N as
   cards.
3. Tick **"border-to-border only"** to restrict country mode to lines whose
   endpoints both land near opposite bbox edges (true country traversals).
4. **⨯ Cancel** stops a running search instantly.
5. Click any card → it highlights on the map. **⬇ GPX** downloads that
   specific candidate as a GPX track.

The **"What do these parameters mean?"** drawer in the side panel explains
every option (`corridor`, `modes`, `alpha`, `score`, ...).

## CLI

### `plan` — one specific A → B

```bash
./target/release/crowfly plan \
  --start "46.17,8.79" --end "47.68,8.62" \
  --width 5000 --modes foot,bike \
  --region switzerland \
  --no-go ./swiss_national_park.geojson \
  --alpha 4 --out ./out
```

Prints a human-readable report (straight km, routed km, detour, max
deviation, mode + surface breakdowns, ferry segments, no-go violations,
elevation ascent/descent if `--elevation-samples > 0`). Writes
`./out/route.geojson`, `./out/route.gpx`, `./out/viewer.html`.

### `explore` — fan from a fixed start

```bash
./target/release/crowfly explore \
  --start "46.17,8.79" \
  --bearing-min 340 --bearing-max 20 --bearing-step 5 \
  --distance-min 80 --distance-max 200 --distance-step 20 \
  --region switzerland \
  --width 5000 --modes foot,bike --top 10
```

`--bearing-min 340 --bearing-max 20` automatically wraps through 0° (so it
sweeps 340 → 350 → 0 → 10 → 20). Writes `explore.html` (clickable card
leaderboard, basemap toggle, per-card GPX) and `top1.gpx`.

### `serve` — interactive web app

```bash
./target/release/crowfly serve \
  --region switzerland --modes foot,bike,mtb \
  --bind 127.0.0.1:7878
```

Builds the graph once at startup (1–2 min for Switzerland), holds it in RAM,
and serves the frontend at the bind address. Each search reuses the same
graph + cached components + R-tree, so request latency is ~1 s for "between"
and ~10–40 s for "country" (depends on candidate count and elevation
samples).

## OSM data

Either point `--osm` at a `.pbf` you already have, or pass `--region` and
crowfly downloads it from [Geofabrik](https://download.geofabrik.de/) on
demand:

| `--region` | Geofabrik URL |
| --- | --- |
| `switzerland` | europe/switzerland-latest.osm.pbf |
| `liechtenstein` | europe/liechtenstein-latest.osm.pbf |
| `france` | europe/france-latest.osm.pbf |
| `europe/great-britain` | europe/great-britain-latest.osm.pbf |
| `north-america/canada` | north-america/canada-latest.osm.pbf |

Bare names default to `europe/`. For other continents pass the full path.
PBFs are cached in `./osm-cache/`.

## Elevation

When `--elevation-samples > 0` (or in the web app whenever
`elevation pts > 0`), crowfly samples `N` points evenly along each route and
queries OpenTopoData's SRTM 30 m endpoint for elevations. Results are cached
in `./elevation-cache/cache.json` keyed by `(lat, lon)` rounded to 4 decimals,
so repeat queries over the same area are free. The free OpenTopoData tier
allows 1 call/sec and 1000 calls/day; one batch covers up to 100 points.

## How it works

1. **Corridor math** (`src/geodesy.rs`) — cross-track / along-track distance
   on a sphere; sub-metre accuracy at country scales, no projection library.
2. **OSM ingestion** (`src/osm.rs`) — two-pass `osmpbfreader`. Pass 1 keeps
   ways whose tags allow at least one requested mode; assigns a surface
   bucket from the `surface` and `tracktype` tags. Pass 2 resolves node
   coordinates. Edges are added to a `petgraph::UnGraph`.
3. **Connected components & spatial index** — at graph build time, union-find
   labels every node with its component, and an `rstar::RTree` indexes node
   coordinates. Both are reused across all subsequent routing queries.
4. **Endpoint snapping** (`Graph::closest_connected_pair`) — for an A → B
   query, take the K=30 nearest nodes to each endpoint via the R-tree, keep
   the cheapest pair whose components match. Catches the common OSM case
   where naïve closest-node would put A and B on different islands.
5. **A\* routing** (`src/route.rs`) — petgraph's `astar` with haversine
   heuristic. Edge cost = `length · (1 + α · (deviation/half_width)²)`;
   deviation is computed at edge-midpoint cross-track at query time, so one
   graph serves many candidate lines (essential for explore + country mode).
6. **Parallel candidates** (`src/explore.rs`) — `evaluate` runs candidate
   A\* searches via rayon; on a multicore box this is the dominant speedup
   for country mode.
7. **Scoring** — `100·(real/ref − 1) + 5·long-edge km + 8·max-dev km`
   (lower = easier). After ranking, deduplicate near-duplicates by
   start+end proximity (8 km separation for country, 0.5 km for between).
8. **Surface + elevation** (`src/viability.rs`, `src/elevation.rs`) — surface
   km per category accumulated from edge tags. Elevation is sampled along
   the routed polyline and fetched once, with on-disk cache.
9. **Output** (`src/output.rs`, `assets/index.html`) — static HTML viewers
   for `plan`/`explore`, an interactive single-page app for `serve`. Both
   embed MapLibre GL JS over CartoDB / OpenTopoMap raster tiles. No API key
   needed for either basemap.

## Common flags

| flag | default | meaning |
| --- | --- | --- |
| `--width` | 5000 | corridor width, metres |
| `--modes` | `foot,bike` (`serve`: `foot,bike,mtb`) | any of `foot`, `bike`, `mtb` |
| `--osm` / `--region` | — | PBF path or Geofabrik key |
| `--cache-dir` | `./osm-cache` | downloaded PBF cache |
| `--elevation-cache` | `./elevation-cache` | elevation lookup cache |
| `--elevation-samples` | 60 (`plan`/`explore`) | per-route samples; 0 disables |
| `--no-go` | — | repeatable; GeoJSON polygons to forbid |
| `--alpha` | 4 | deviation penalty: 0 = shortest path, ~10 = sticks tightly |
| `--long-edge-threshold` | 800 | metres — flags long single edges |
| `--out` | `./out` | output directory (`plan`/`explore`) |
| `--bind` (`serve`) | `127.0.0.1:7878` | server address |

## Tests

```bash
cargo test --release
```

Unit tests cover the geodesic primitives (haversine, cross-track, corridor
predicate).

## Limitations

- Spherical earth, not WGS84 ellipsoid (~0.3 % distance error).
- A\* runs on the raw OSM graph without preprocessing — long mountain routes
  on a country-sized graph still take a few seconds each. Contraction
  Hierarchies / ALT landmarks would unlock another 10–100×.
- Mode classification is heuristic OSM tag matching, not the full
  Valhalla/OSRM ruleset.
- Switzerland-sized PBFs need ~1 GB RAM during graph construction; the
  resulting in-memory graph for foot+bike+mtb is ~16 M nodes.
- OpenTopoData free tier rate-limits elevation lookups to 1 batch/sec.
- The `serve` mode is single-process, single-user — designed for `localhost`,
  not internet exposure.

## License

MIT.
