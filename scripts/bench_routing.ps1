# Pure-routing latency benchmark against a running `crowfly serve`.
# Sends /api/between requests with elevation_samples=0 so the timing
# isolates graph routing (no DEM / network work), repeats each case and
# reports min/median. Usage:
#   pwsh scripts/bench_routing.ps1 -Label before [-Reps 3]
param(
    [string]$Host_ = "http://127.0.0.1:7878",
    [string]$Label = "bench",
    [int]$Reps = 3
)

$cases = @(
    @{ name = "zh_winti";  start = @(47.3769, 8.5417); end = @(47.4995, 8.7240); width = 6000 },
    @{ name = "bn_fr";     start = @(46.9481, 7.4474); end = @(46.8060, 7.1614); width = 8000 },
    @{ name = "ls_vv";     start = @(46.5197, 6.6323); end = @(46.4628, 6.8419); width = 5000 },
    @{ name = "zh_lu";     start = @(47.3769, 8.5417); end = @(47.0502, 8.3093); width = 8000 },
    @{ name = "bn_thn";    start = @(46.9481, 7.4474); end = @(46.7494, 7.6280); width = 6000 },
    # Country-scale crossings: the heavy regime for the A* core.
    @{ name = "ls_zh";     start = @(46.5197, 6.6323); end = @(47.3769, 8.5417); width = 12000 },
    @{ name = "bs_chur";   start = @(47.5596, 7.5886); end = @(46.8508, 9.5320); width = 14000 }
)

$out = @()
foreach ($c in $cases) {
    $body = @{
        start = $c.start; end = $c.end; modes = "foot,bike"
        width_m = $c.width; alternatives = 1; elevation_samples = 0
    } | ConvertTo-Json -Compress
    $times = @()
    $realKm = 0.0
    for ($i = 0; $i -lt $Reps; $i++) {
        $t0 = [System.Diagnostics.Stopwatch]::StartNew()
        try {
            $resp = Invoke-RestMethod -Uri "$Host_/api/between" -Method Post -Body $body -ContentType "application/json" -TimeoutSec 600
        } catch {
            Write-Host ("  {0}: ERROR {1}" -f $c.name, $_.Exception.Message)
            continue
        }
        $t0.Stop()
        $times += $t0.Elapsed.TotalSeconds
        if ($resp.results.Count -gt 0) { $realKm = $resp.results[0].real_km }
    }
    if ($times.Count -eq 0) { continue }
    $sorted = $times | Sort-Object
    $min = $sorted[0]
    $med = $sorted[[math]::Floor($sorted.Count / 2)]
    Write-Host ("  {0,-10} {1,7:F1} km   min {2,6:F2}s   med {3,6:F2}s" -f $c.name, $realKm, $min, $med)
    $out += [pscustomobject]@{ name = $c.name; real_km = $realKm; min_s = $min; med_s = $med; times = $times }
}
$total = ($out | Measure-Object -Property min_s -Sum).Sum
Write-Host ("  {0,-10} {1,18}   sum-of-min {2,6:F2}s" -f "TOTAL", "", $total)
$out | ConvertTo-Json -Depth 4 | Set-Content ("experiments/bench-{0}.json" -f $Label)
