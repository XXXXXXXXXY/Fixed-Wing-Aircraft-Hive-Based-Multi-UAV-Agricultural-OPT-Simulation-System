param(
    [string]$Plan = "planexample\test1.plan",
    [string]$Scenario = "configs\test1_from_qgc_fence.json",
    [string]$OptVisualPlan = "configs\opt_visual_plan.json",
    [ValidateSet("scout", "work", "hybrid")]
    [string]$Mode = "hybrid",
    [int]$MaxScouts = 8,
    [int]$DroneCount = 8,
    [int]$FixedWingCount = 1,
    [double]$HiveSpeedKmh = 30.0,
    [int]$OptSteps = 5000,
    [double]$DurationS = 0.0,
    [switch]$NoQGC
)

$ErrorActionPreference = "Stop"

$workspace = (Resolve-Path ".").Path
$qgc = "C:\Program Files\QGroundControl\bin\QGroundControl.exe"

if (Test-Path $Plan) {
    python (Join-Path $workspace "scripts\qgc_plan_to_scenario.py") $Plan -o $Scenario
}

if ($Mode -ne "scout") {
    $exe = Join-Path $workspace "scout_opt.exe"
    if (-not (Test-Path $exe)) {
        $build = Join-Path $workspace "build.ps1"
        powershell -ExecutionPolicy Bypass -File $build
    }
    & $exe --scenario $Scenario --fixed-wing --steps $OptSteps --export-visual $OptVisualPlan
    $Scenario = $OptVisualPlan
}

if (-not $NoQGC) {
    if (Test-Path $qgc) {
        Start-Process -FilePath $qgc
        Start-Sleep -Seconds 4
    } else {
        Write-Warning "QGroundControl not found at $qgc. Start QGC manually, then run the bridge command below."
    }
}

python -u `
    (Join-Path $workspace "scripts\qgc_demo_bridge.py") `
    $Scenario `
    --mode $Mode `
    --max-scouts $MaxScouts `
    --drone-count $DroneCount `
    --fixed-wing-count $FixedWingCount `
    --hive-speed-kmh $HiveSpeedKmh `
    --duration-s $DurationS
