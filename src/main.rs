use anyhow::{anyhow, Context, Result};
use clap::{Parser, Subcommand};
use std::path::PathBuf;

use crowfly::elevation::ElevationClient;
use crowfly::explore;
use crowfly::fetch;
use crowfly::geodesy::{haversine, in_corridor, LatLon};
use crowfly::osm::{self, BBox, MODE_BIKE, MODE_FOOT, MODE_MTB};
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

#[derive(Parser, Debug)]
struct ServeArgs {
    /// OSM .pbf file. Mutually exclusive with --region.
    #[arg(long)]
    osm: Option<PathBuf>,
    /// Geofabrik region key (downloaded if missing).
    #[arg(long)]
    region: Option<String>,
    /// Cache dir for downloaded PBFs.
    #[arg(long, default_value = "./osm-cache")]
    cache_dir: PathBuf,
    /// Cache dir for elevation lookups.
    #[arg(long, default_value = "./elevation-cache")]
    elevation_cache: PathBuf,
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
    /// Total corridor width, metres.
    #[arg(long, default_value_t = 5000.0)]
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
    /// Elevation samples per route. 0 disables elevation lookup.
    #[arg(long, default_value_t = 60)]
    elevation_samples: usize,
    /// Cache dir for downloaded elevation lookups.
    #[arg(long, default_value = "./elevation-cache")]
    elevation_cache: PathBuf,
    /// Output directory.
    #[arg(long, default_value = "./out")]
    out: PathBuf,
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

fn parse_modes(s: &str) -> Result<u8> {
    let mut m = 0u8;
    for tok in s.split(',') {
        match tok.trim().to_ascii_lowercase().as_str() {
            "foot" | "walk" | "hike" => m |= MODE_FOOT,
            "bike" | "cycle" | "bicycle" => m |= MODE_BIKE,
            "mtb" | "gravel" => m |= MODE_MTB,
            "" => {}
            other => return Err(anyhow!("unknown mode: {other}")),
        }
    }
    if m == 0 {
        return Err(anyhow!("no modes selected"));
    }
    Ok(m)
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

fn run_plan(args: PlanArgs) -> Result<()> {
    let start = parse_latlon(&args.start)?;
    let end = parse_latlon(&args.end)?;
    let modes = parse_modes(&args.common.modes)?;
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

    let (route, report): (RouteResult, Report) = if let Some(pbf) = pbf {
        let bbox = corridor_bbox(start, end, 0.10);
        let total_len_m = haversine(start, end);
        let keep = |p0: LatLon, p1: LatLon| -> bool {
            in_corridor(p0, start, end, half, total_len_m)
                || in_corridor(p1, start, end, half, total_len_m)
        };
        let graph = osm::build(&pbf, bbox, modes, keep)?;
        if graph.graph.node_count() == 0 {
            return Err(anyhow!(
                "no graph nodes in corridor — the PBF may not cover this area"
            ));
        }
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
        (r, rep)
    } else {
        println!("\n(no --osm or --region: emitting corridor + reference line only)");
        let preview = RouteResult {
            points: vec![start, end],
            edges: vec![],
            total_length_m: 0.0,
        };
        let report = viability::analyse(
            &preview,
            start,
            end,
            &no_go,
            args.common.long_edge_threshold,
        );
        (preview, report)
    };

    let elevation = if args.common.elevation_samples > 0 && route.points.len() >= 2 {
        println!(
            "\nFetching elevation profile ({} samples)…",
            args.common.elevation_samples
        );
        let mut client = ElevationClient::new(&args.common.elevation_cache)?;
        let p = client.profile(&route.points, args.common.elevation_samples);
        let _ = client.save();
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
    let modes = parse_modes(&args.common.modes)?;
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
    let graph = osm::build(&pbf, bbox, modes, |_, _| true)?;
    if graph.graph.node_count() == 0 {
        return Err(anyhow!("no graph nodes in search area"));
    }

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

    let mut ranked = explore::rank(viable);
    ranked.truncate(args.top);

    if args.common.elevation_samples > 0 {
        println!(
            "\nFetching elevation profiles ({} samples per route)…",
            args.common.elevation_samples
        );
        let mut client = ElevationClient::new(&args.common.elevation_cache)?;
        explore::enrich_with_elevation(&mut ranked, &mut client, args.common.elevation_samples);
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
    use crowfly::osm::BBox;
    let modes = parse_modes(&args.modes)?;
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
    // Loose lat/lon bbox lets us load anything; the graph dimensions then
    // determine the real area used for country sampling.
    let loose_bbox = BBox {
        lat_min: -85.0,
        lat_max: 85.0,
        lon_min: -180.0,
        lon_max: 180.0,
    };

    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()?;
    rt.block_on(async move {
        let pbf_for_build = pbf.clone();
        let graph = tokio::task::spawn_blocking(move || -> Result<crowfly::osm::Graph> {
            eprintln!("Building graph from {} …", pbf_for_build.display());
            crowfly::osm::build(&pbf_for_build, loose_bbox, modes, |_, _| true)
        })
        .await??;
        let real_bbox = graph_bbox(&graph);
        eprintln!(
            "Graph ready: {} nodes; bbox [{:.3},{:.3}]×[{:.3},{:.3}]",
            graph.graph.node_count(),
            real_bbox.lat_min,
            real_bbox.lat_max,
            real_bbox.lon_min,
            real_bbox.lon_max
        );
        let elev = crowfly::elevation::ElevationClient::new(&args.elevation_cache)?;
        let state = crowfly::server::AppState(std::sync::Arc::new(
            crowfly::server::ServerState {
                graph,
                bbox: real_bbox,
                no_go,
                long_edge_threshold_m: args.long_edge_threshold,
                elevation_cache: args.elevation_cache.clone(),
                elev: std::sync::Mutex::new(elev),
                progress: std::sync::Mutex::new(None),
            },
        ));
        crowfly::server::serve(state, &args.bind).await
    })
}

fn graph_bbox(g: &crowfly::osm::Graph) -> crowfly::osm::BBox {
    use crowfly::osm::BBox;
    let mut lat_min = f64::INFINITY;
    let mut lat_max = f64::NEG_INFINITY;
    let mut lon_min = f64::INFINITY;
    let mut lon_max = f64::NEG_INFINITY;
    for ni in g.graph.node_indices() {
        let p = g.graph[ni];
        if p.lat < lat_min { lat_min = p.lat; }
        if p.lat > lat_max { lat_max = p.lat; }
        if p.lon < lon_min { lon_min = p.lon; }
        if p.lon > lon_max { lon_max = p.lon; }
    }
    BBox { lat_min, lat_max, lon_min, lon_max }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Plan(a) => run_plan(a),
        Cmd::Explore(a) => run_explore(a),
        Cmd::Serve(a) => run_serve(a),
    }
}
