use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand};
use std::path::PathBuf;

use crowfly::dem::{self, Dem};
use crowfly::elevation::{profile_from_dem, ElevationClient};
use crowfly::explore;
use crowfly::fetch;
use crowfly::geodesy::{haversine, in_corridor, LatLon};
use crowfly::graph_cache;
use crowfly::osm::{self, BBox, Graph, MODE_BIKE, MODE_FOOT, MODE_MTB};
use crowfly::output::{ExploreOutputs, Outputs};
use crowfly::route::{self, RouteParams, RouteResult};
use crowfly::viability::{self, NoGoZone, Report};

/// Plan and validate "azimut nord"-style straight-line crossings.
#[derive(Parser, Debug)]
#[command(version, about)]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Plan a single line A → B and validate it.
    Plan(PlanArgs),
    /// Search a fan of candidate lines from a fixed start and rank by viability.
    Explore(ExploreArgs),
    /// Run the interactive web app on localhost.
    Serve(ServeArgs),
}

/// DEM options, shared by every subcommand so `plan` / `explore` route on the
/// same slope-aware graph as `serve`.
#[derive(Parser, Debug, Clone)]
struct DemArgs {
    /// Directory holding SRTM `.hgt` tiles (NxxEyyy.hgt). If empty, the named
    /// `--dem-pack` is downloaded from viewfinderpanoramas.org on first run.
    /// Every graph node is then stamped with its elevation, and the router
    /// penalises descending edges on non-rideable surfaces (steps / footway /
    /// path) so bike+hike combos prefer a rideable line on the way down. The
    /// same tiles provide route elevation profiles.
    #[arg(long, default_value = "./dem-cache")]
    dem_dir: PathBuf,
    /// viewfinderpanoramas DEM3 pack(s) to download, comma-separated. Each
    /// pack covers a 6°×4° square: `L32` is N44–48 / E6–12 (most of
    /// Switzerland and the Alps); `L31` is N44–48 / E0–6 (Geneva, Jura).
    /// Already-fetched packs are skipped via a marker file in `--dem-dir`.
    /// Set to an empty string to disable DEM loading entirely (no slope-aware
    /// routing; elevation falls back to OpenTopoData in the CLI and is
    /// omitted by `serve`). See <https://viewfinderpanoramas.org/dem3.html>
    /// for the world grid.
    #[arg(long, default_value = "L31,L32")]
    dem_pack: String,
}

#[derive(Parser, Debug)]
struct ServeArgs {
    /// OSM .pbf file. Mutually exclusive with --region.
    #[arg(long)]
    osm: Option<PathBuf>,
    /// Geofabrik region key (downloaded if missing).
    #[arg(long)]
    region: Option<String>,
    /// Cache dir for downloaded PBFs and the built-graph cache
    /// (`<pbf name>.graph`).
    #[arg(long, default_value = "./osm-cache")]
    cache_dir: PathBuf,
    /// Always build the graph from the PBF instead of reusing the cache in
    /// --cache-dir. The cache is rebuilt automatically when the PBF, the
    /// modes or the graph-building code change.
    #[arg(long)]
    no_graph_cache: bool,
    #[command(flatten)]
    dem: DemArgs,
    /// Repeatable: GeoJSON polygon files to forbid.
    #[arg(long)]
    no_go: Vec<PathBuf>,
    /// Modes used to filter the OSM graph at startup. Searches further
    /// restrict modes per request.
    #[arg(long, default_value = "foot,bike,mtb")]
    modes: String,
    /// Long-edge flagging threshold (metres) — likely ferries / graph gaps.
    #[arg(long, default_value_t = 800.0)]
    long_edge_threshold: f64,
    /// Bind address.
    #[arg(long, default_value = "127.0.0.1:7878")]
    bind: String,
}

#[derive(Parser, Debug)]
struct CommonArgs {
    /// Total corridor width, metres (the web app's default too).
    #[arg(long, default_value_t = 8000.0)]
    width: f64,
    /// Allowed transport modes, comma-separated: foot, bike, mtb.
    #[arg(long, default_value = "foot,bike")]
    modes: String,
    /// OSM .pbf file. If omitted, --region triggers a download.
    #[arg(long)]
    osm: Option<PathBuf>,
    /// Geofabrik region key, e.g. "switzerland", "europe/switzerland",
    /// "north-america/canada". The matching .pbf is downloaded into
    /// --cache-dir and reused on subsequent runs.
    #[arg(long)]
    region: Option<String>,
    /// Where to keep cached PBFs.
    #[arg(long, default_value = "./osm-cache")]
    cache_dir: PathBuf,
    /// Repeatable: GeoJSON polygon files that the route must avoid.
    #[arg(long)]
    no_go: Vec<PathBuf>,
    /// Deviation penalty strength: 0 = pure shortest path, ~10 = sticks tightly.
    #[arg(long, default_value_t = 4.0)]
    alpha: f64,
    /// Restrict routing to paved surfaces only (asphalt/concrete/paving stones).
    /// Skips gravel, paths, unpaved tracks, and ferries.
    #[arg(long, default_value_t = false)]
    paved_only: bool,
    /// Long-edge flagging threshold (metres) — likely ferries / graph gaps.
    #[arg(long, default_value_t = 800.0)]
    long_edge_threshold: f64,
    /// Elevation chart samples per route; 0 disables elevation. Profiles come
    /// from the DEM tiles when loaded (as in `serve`), else from OpenTopoData.
    #[arg(long, default_value_t = 60)]
    elevation_samples: usize,
    /// Cache dir for OpenTopoData lookups (only used without a DEM).
    #[arg(long, default_value = "./elevation-cache")]
    elevation_cache: PathBuf,
    /// Output directory.
    #[arg(long, default_value = "./out")]
    out: PathBuf,
    #[command(flatten)]
    dem: DemArgs,
}

#[derive(Parser, Debug)]
struct PlanArgs {
    /// Start "lat,lon".
    #[arg(long)]
    start: String,
    /// End "lat,lon".
    #[arg(long)]
    end: String,
    #[command(flatten)]
    common: CommonArgs,
}

#[derive(Parser, Debug)]
struct ExploreArgs {
    /// Fixed start "lat,lon".
    #[arg(long)]
    start: String,
    /// Search bearings (compass degrees), inclusive.
    #[arg(long, default_value_t = 320.0)]
    bearing_min: f64,
    #[arg(long, default_value_t = 40.0)]
    bearing_max: f64,
    #[arg(long, default_value_t = 5.0)]
    bearing_step: f64,
    /// Candidate line lengths, km.
    #[arg(long, default_value_t = 80.0)]
    distance_min: f64,
    #[arg(long, default_value_t = 200.0)]
    distance_max: f64,
    #[arg(long, default_value_t = 20.0)]
    distance_step: f64,
    /// Keep this many top candidates.
    #[arg(long, default_value_t = 10)]
    top: usize,
    #[command(flatten)]
    common: CommonArgs,
}

fn parse_latlon(s: &str) -> Result<LatLon> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 2 {
        return Err(anyhow!("expected 'lat,lon', got {s:?}"));
    }
    let lat: f64 = parts[0].trim().parse().context("parsing lat")?;
    let lon: f64 = parts[1].trim().parse().context("parsing lon")?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err(anyhow!("lat/lon out of range"));
    }
    Ok(LatLon::new(lat, lon))
}

/// Resolve --osm / --region into a PBF path, downloading if needed.
fn resolve_pbf(common: &CommonArgs) -> Result<Option<PathBuf>> {
    if let Some(p) = &common.osm {
        return Ok(Some(p.clone()));
    }
    if let Some(region) = &common.region {
        return Ok(Some(fetch::ensure_pbf(region, &common.cache_dir)?));
    }
    Ok(None)
}

fn load_no_go(common: &CommonArgs) -> Result<Vec<NoGoZone>> {
    let zones: Vec<NoGoZone> = common
        .no_go
        .iter()
        .map(|p| viability::load_no_go(p))
        .collect::<Result<_>>()?;
    if !zones.is_empty() {
        println!(
            "Loaded {} no-go zone(s): {}",
            zones.len(),
            zones
                .iter()
                .map(|z| z.name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    Ok(zones)
}

fn corridor_bbox(a: LatLon, b: LatLon, pad_deg: f64) -> BBox {
    BBox {
        lat_min: a.lat.min(b.lat) - pad_deg,
        lat_max: a.lat.max(b.lat) + pad_deg,
        lon_min: a.lon.min(b.lon) - pad_deg,
        lon_max: a.lon.max(b.lon) + pad_deg,
    }
}

fn print_modes(modes: u8) {
    println!(
        "Modes    : {}{}{}",
        if modes & MODE_FOOT != 0 { "foot " } else { "" },
        if modes & MODE_BIKE != 0 { "bike " } else { "" },
        if modes & MODE_MTB != 0 { "mtb " } else { "" }
    );
}

/// Load the DEM packs named by `--dem-pack` (downloading missing ones), or
/// None when disabled or no tiles were found.
fn load_dem(args: &DemArgs) -> Result<Option<Dem>> {
    let packs: Vec<&str> = args
        .dem_pack
        .split(',')
        .map(|s| s.trim())
        .filter(|s| !s.is_empty())
        .collect();
    if packs.is_empty() {
        return Ok(None);
    }
    dem::ensure_packs(&args.dem_dir, &packs)?;
    let d = Dem::load_dir(&args.dem_dir)?;
    if d.is_empty() {
        eprintln!("  (no tiles found — slope-aware routing and DEM elevation disabled)");
        return Ok(None);
    }
    eprintln!("  loaded {} tile(s)", d.tiles.len());
    Ok(Some(d))
}

/// Stamp (and smooth) node elevations for slope-aware routing, then keep
/// only the tiles under the graph for elevation profiles — routes never
/// leave its bbox. Every subcommand goes through here so they all route on
/// the same graph.
fn attach_dem(graph: &mut Graph, dem: Option<Dem>) -> Option<Dem> {
    let mut d = dem?;
    graph.stamp_elevations(&d);
    graph.smooth_elevations(2);
    d.retain_bbox(graph.bbox());
    eprintln!(
        "  keeping {} DEM tile(s) for elevation profiles",
        d.tiles.len()
    );
    Some(d)
}

fn run_plan(args: PlanArgs) -> Result<()> {
    let start = parse_latlon(&args.start)?;
    let end = parse_latlon(&args.end)?;
    let modes = osm::parse_modes(&args.common.modes)?;
    let total_km = haversine(start, end) / 1000.0;
    let half = args.common.width / 2.0;

    println!("Mode     : plan");
    println!("Start    : {:.5}, {:.5}", start.lat, start.lon);
    println!("End      : {:.5}, {:.5}", end.lat, end.lon);
    println!("Straight : {:.2} km", total_km);
    println!("Corridor : {:.0} m wide", args.common.width);
    print_modes(modes);

    let no_go = load_no_go(&args.common)?;
    let pbf = resolve_pbf(&args.common)?;

    let (route, report, dem): (RouteResult, Report, Option<Dem>) = if let Some(pbf) = pbf {
        let bbox = corridor_bbox(start, end, 0.10);
        let total_len_m = haversine(start, end);
        let keep = |p0: LatLon, p1: LatLon| -> bool {
            in_corridor(p0, start, end, half, total_len_m)
                || in_corridor(p1, start, end, half, total_len_m)
        };
        let mut graph = osm::build(&pbf, bbox, modes, keep)?;
        if graph.graph.node_count() == 0 {
            return Err(anyhow!(
                "no graph nodes in corridor — the PBF may not cover this area"
            ));
        }
        let dem = attach_dem(&mut graph, load_dem(&args.common.dem)?);
        let (start_idx, end_idx, sd, ed) = graph
            .closest_connected_pair(start, end)
            .ok_or_else(|| anyhow!("graph empty"))?;
        println!(
            "Snapped to a connected component: start {:.0} m, end {:.0} m",
            sd, ed
        );
        let params = RouteParams {
            modes,
            half_width_m: half,
            alpha: args.common.alpha,
            corridor_max_m: half,
            paved_only: args.common.paved_only,
        };
        let r = route::shortest(&graph, start_idx, end_idx, start, end, &params)?;
        let rep = viability::analyse(&r, start, end, &no_go, args.common.long_edge_threshold);
        viability::print_report(&rep, total_km);
        (r, rep, dem)
    } else {
        println!("\n(no --osm or --region: emitting corridor + reference line only)");
        let preview = RouteResult {
            points: vec![start, end],
            edges: vec![],
            total_length_m: 0.0,
            node_indices: vec![],
        };
        let report = viability::analyse(
            &preview,
            start,
            end,
            &no_go,
            args.common.long_edge_threshold,
        );
        (preview, report, None)
    };

    let n_samples = args.common.elevation_samples;
    let elevation = if n_samples > 0 && route.points.len() >= 2 {
        let p = match &dem {
            Some(d) => {
                println!("\nElevation profile from the DEM ({n_samples} samples)");
                profile_from_dem(d, &route.points, n_samples)
            }
            None => {
                println!("\nFetching elevation profile ({n_samples} samples)…");
                let mut client = ElevationClient::new(&args.common.elevation_cache)?;
                let p = client.profile(&route.points, n_samples);
                let _ = client.save();
                p
            }
        };
        if let Some(prof) = &p {
            println!(
                "Ascent: {:.0} m, Descent: {:.0} m, Max: {:.0} m, Min: {:.0} m",
                prof.ascent_m, prof.descent_m, prof.max_m, prof.min_m
            );
        }
        p
    } else {
        None
    };

    let outputs = Outputs {
        start,
        end,
        corridor_width_m: args.common.width,
        route: &route,
        no_go: &no_go,
        report: &report,
        elevation: elevation.as_ref(),
    };
    outputs.write_all(&args.common.out)?;
    println!(
        "\nWrote {}/route.geojson, route.gpx, viewer.html",
        args.common.out.display()
    );
    Ok(())
}

fn run_explore(args: ExploreArgs) -> Result<()> {
    let start = parse_latlon(&args.start)?;
    let modes = osm::parse_modes(&args.common.modes)?;
    let half = args.common.width / 2.0;

    println!("Mode     : explore");
    println!("Start    : {:.5}, {:.5}", start.lat, start.lon);
    println!(
        "Bearings : {}°..{}° step {}°",
        args.bearing_min, args.bearing_max, args.bearing_step
    );
    println!(
        "Distance : {} km..{} km step {} km",
        args.distance_min, args.distance_max, args.distance_step
    );
    println!("Corridor : {:.0} m wide", args.common.width);
    print_modes(modes);

    let candidates = explore::generate_candidates(
        start,
        args.bearing_min,
        args.bearing_max,
        args.bearing_step,
        args.distance_min,
        args.distance_max,
        args.distance_step,
    );
    println!("Generated {} candidate lines", candidates.len());

    let no_go = load_no_go(&args.common)?;
    let pbf = resolve_pbf(&args.common)?
        .ok_or_else(|| anyhow!("explore mode needs --osm or --region"))?;

    // Build one big graph covering all candidate corridors. We use the
    // enclosing bbox as the cheap area filter; the deviation cost in A* will
    // pull each route toward its own line.
    let bbox = explore::enclosing_bbox(start, &candidates, 0.10);
    let mut graph = osm::build(&pbf, bbox, modes, |_, _| true)?;
    if graph.graph.node_count() == 0 {
        return Err(anyhow!("no graph nodes in search area"));
    }
    let dem = attach_dem(&mut graph, load_dem(&args.common.dem)?);

    let params = RouteParams {
        modes,
        half_width_m: half,
        alpha: args.common.alpha,
        corridor_max_m: half,
        paved_only: args.common.paved_only,
    };
    let filter = explore::RouteFilter::lax(half.max(2_000.0));
    println!(
        "Routing {} candidates on a graph of {} nodes…",
        candidates.len(),
        graph.graph.node_count()
    );
    let results = explore::evaluate(
        &graph,
        candidates,
        &no_go,
        &params,
        &filter,
        args.common.long_edge_threshold,
        None,
        None,
    );

    let mut viable: Vec<explore::Scored> = Vec::new();
    let mut rejected = 0usize;
    for r in results {
        match r {
            Ok(s) => viable.push(s),
            Err(_) => rejected += 1,
        }
    }
    println!(
        "{} viable, {} rejected (no-route or no-go)",
        viable.len(),
        rejected
    );
    if viable.is_empty() {
        return Err(anyhow!(
            "no candidate produced a viable route. Widen --width or relax modes."
        ));
    }

    let n_samples = args.common.elevation_samples;
    if let (Some(d), true) = (&dem, n_samples > 0) {
        // As in `serve`: DEM profiles are cheap, so every viable line gets
        // one and the ascent term takes part in the ranking.
        for s in viable.iter_mut() {
            s.elevation = profile_from_dem(d, &s.route.points, n_samples);
            explore::rescore_with_elevation(s);
        }
    }
    let mut ranked = explore::rank(viable);
    ranked.truncate(args.top);

    if dem.is_none() && n_samples > 0 {
        println!("\nFetching elevation profiles ({n_samples} samples per route)…");
        let mut client = ElevationClient::new(&args.common.elevation_cache)?;
        explore::enrich_with_elevation(&mut ranked, &mut client, n_samples);
    }

    explore::print_leaderboard(&ranked, start);

    let exp = ExploreOutputs {
        start,
        corridor_width_m: args.common.width,
        ranked: &ranked,
        no_go: &no_go,
    };
    exp.write_all(&args.common.out)?;
    println!(
        "\nWrote {}/explore.geojson, explore.html, top1.gpx",
        args.common.out.display()
    );
    Ok(())
}

fn run_serve(args: ServeArgs) -> Result<()> {
    let modes = osm::parse_modes(&args.modes)?;
    let pbf = if let Some(p) = &args.osm {
        p.clone()
    } else if let Some(region) = &args.region {
        fetch::ensure_pbf(region, &args.cache_dir)?
    } else {
        return Err(anyhow!("--osm or --region required"));
    };
    let no_go: Vec<_> = args
        .no_go
        .iter()
        .map(|p| crowfly::viability::load_no_go(p))
        .collect::<Result<_>>()?;
    let graph_cache_dir = (!args.no_graph_cache).then(|| args.cache_dir.clone());

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        let dem_args = args.dem.clone();
        let (graph, dem) = tokio::task::spawn_blocking(move || -> Result<(Graph, Option<Dem>)> {
            eprintln!("Loading graph for {} …", pbf.display());
            let mut g = graph_cache::build_or_load(&pbf, modes, graph_cache_dir.as_deref())?;
            let dem = attach_dem(&mut g, load_dem(&dem_args)?);
            Ok((g, dem))
        })
        .await??;
        let state = crowfly::server::ServerState::new(graph, no_go, args.long_edge_threshold, dem);
        let b = state.bbox;
        eprintln!(
            "Graph ready: {} nodes; bbox [{:.3},{:.3}]×[{:.3},{:.3}]; {} outline points",
            state.graph.graph.node_count(),
            b.lat_min,
            b.lat_max,
            b.lon_min,
            b.lon_max,
            state.outline.len()
        );
        let state = crowfly::server::AppState(std::sync::Arc::new(state));
        crowfly::server::serve(state, &args.bind).await
    })
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Plan(a) => run_plan(a),
        Cmd::Explore(a) => run_explore(a),
        Cmd::Serve(a) => run_serve(a),
    }
}
