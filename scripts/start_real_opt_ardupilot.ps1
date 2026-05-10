param(
    [string]$Plan = "planexample\test1.plan",
    [string]$Scenario = "configs\test1_from_qgc_fence.json",
    [string]$OptVisualPlan = "configs\opt_visual_plan.json",
    [int]$Count = 8,
    [int]$PlaneCount = 1,
    [int]$RoverCount = 1,
    [int]$Steps = 5000,
    [switch]$NoQGC
)

$ErrorActionPreference = "Stop"
$workspace = (Resolve-Path ".").Path

python (Join-Path $workspace "scripts\qgc_plan_to_scenario.py") $Plan -o $Scenario

$exe = Join-Path $workspace "scout_opt.exe"
if (-not (Test-Path $exe)) {
    powershell -ExecutionPolicy Bypass -File (Join-Path $workspace "build.ps1")
}

& $exe --scenario $Scenario --fixed-wing --steps $Steps --export-visual $OptVisualPlan
if ($LASTEXITCODE -ne 0) {
    Write-Error "C OPT failed."
}

$args = @(
    "-ExecutionPolicy", "Bypass",
    "-File", (Join-Path $workspace "scripts\start_real_ardupilot.ps1"),
    "-Count", "$Count",
    "-PlaneCount", "$PlaneCount",
    "-RoverCount", "$RoverCount",
    "-VisualPlan", $OptVisualPlan
)
if ($NoQGC) {
    $args += "-NoQGC"
}
powershell @args
