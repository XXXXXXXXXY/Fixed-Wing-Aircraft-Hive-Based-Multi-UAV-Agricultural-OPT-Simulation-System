param(
    [int]$Count = 1,
    [int]$PlaneCount = 0,
    [int]$RoverCount = 0,
    [string]$VisualPlan = "",
    [switch]$NoQGC,
    [switch]$NoRouter
)

$ErrorActionPreference = "Stop"

$currentPath = $env:Path
[System.Environment]::SetEnvironmentVariable("PATH", $null, "Process")
$env:Path = "$currentPath;C:\msys64\usr\bin"

$workspace = (Resolve-Path ".").Path
$copterBinary = Join-Path $workspace "ardupilot\build\sitl\bin\arducopter.exe"
$planeBinary = Join-Path $workspace "ardupilot\build\sitl\bin\arduplane.exe"
$roverBinary = Join-Path $workspace "ardupilot\build\sitl\bin\ardurover.exe"
$defaults = Join-Path $workspace "configs\sitl_qgc_relaxed.parm"
$runRoot = Join-Path $workspace ".tmp\real_sitl"
$qgc = "C:\Program Files\QGroundControl\bin\QGroundControl.exe"

function Convert-LocalToLatLon($origin, $point) {
    $lat0 = [double]$origin.lat
    $lon0 = [double]$origin.lon
    $lat = $lat0 + ([double]$point.y / 111111.0)
    $lon = $lon0 + ([double]$point.x / (111111.0 * [Math]::Cos($lat0 * [Math]::PI / 180.0)))
    return @{ Lat = $lat; Lon = $lon }
}

$visual = $null
$copterHome = @{ Lat = 32.0858944; Lon = 118.8988587; Alt = 20.0; Heading = 0.0 }
$planeHome = @{ Lat = 32.0778000; Lon = 118.8868000; Alt = 30.0; Heading = 0.0 }
$roverHome = @{ Lat = 32.0858944; Lon = 118.8988587; Alt = 20.0; Heading = 0.0 }
if ($VisualPlan -ne "" -and (Test-Path $VisualPlan)) {
    $visual = Get-Content $VisualPlan -Raw | ConvertFrom-Json
    if ($visual.origin -and $visual.hive.stops.Count -gt 0) {
        $hiveStart = Convert-LocalToLatLon $visual.origin $visual.hive.stops[0]
        $copterHome.Lat = $hiveStart.Lat
        $copterHome.Lon = $hiveStart.Lon
        $roverHome.Lat = $hiveStart.Lat
        $roverHome.Lon = $hiveStart.Lon
    }
    if ($visual.origin -and $visual.fixed_wing.airport) {
        $airport = Convert-LocalToLatLon $visual.origin $visual.fixed_wing.airport
        $planeHome.Lat = $airport.Lat
        $planeHome.Lon = $airport.Lon
    }
}

if (-not (Test-Path $copterBinary)) {
    Write-Error "ArduCopter SITL binary not found: $copterBinary"
}
if ($PlaneCount -gt 0 -and -not (Test-Path $planeBinary)) {
    Write-Error "ArduPlane SITL binary not found: $planeBinary. Run scripts\build_ardupilot_sitl.ps1 -Targets plane"
}
if ($RoverCount -gt 0 -and -not (Test-Path $roverBinary)) {
    Write-Error "ArduRover SITL binary not found: $roverBinary. Run scripts\build_ardupilot_sitl.ps1 -Targets rover"
}

$oldArdu = @(Get-Process arducopter,arduplane,ardurover -ErrorAction SilentlyContinue)
if ($oldArdu.Count -gt 0) {
    Stop-Process -Id ($oldArdu | Select-Object -ExpandProperty Id) -Force -ErrorAction SilentlyContinue
}

$oldRouters = @(Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match "python" -and $_.CommandLine -like "*ardupilot_real_router.py*"
})
if ($oldRouters.Count -gt 0) {
    Stop-Process -Id ($oldRouters | Select-Object -ExpandProperty ProcessId) -Force -ErrorAction SilentlyContinue
}

New-Item -ItemType Directory -Force -Path $runRoot | Out-Null

for ($i = 0; $i -lt $Count; $i++) {
    $dir = Join-Path $runRoot "copter_$i"
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $sysid = $i + 1
    $lat = $copterHome.Lat
    $lon = $copterHome.Lon
    $homeArg = ("{0:F7},{1:F7},20,0" -f $lat, $lon)
    $argsList = @(
        "--model", "quad",
        "--speedup", "1",
        "-I", "$i",
        "--home", $homeArg,
        "--sysid", "$sysid",
        "--serial0", "tcp:0",
        "--sim-address=127.0.0.1",
        "--defaults", $defaults
    )
    Start-Process `
        -FilePath $copterBinary `
        -ArgumentList $argsList `
        -WorkingDirectory $dir `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $dir "arducopter.out.log") `
        -RedirectStandardError (Join-Path $dir "arducopter.err.log")
    $tcpPort = 5760 + $i * 10
    Write-Host "started real ArduPilot SITL sysid=$sysid tcp=$tcpPort home=$homeArg"
}

for ($i = 0; $i -lt $PlaneCount; $i++) {
    $dir = Join-Path $runRoot "plane_$i"
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $sysid = 100 + $i
    $instance = 10 + $i
    $lat = $planeHome.Lat
    $lon = $planeHome.Lon + ($i * 0.00018)
    $homeArg = ("{0:F7},{1:F7},30,0" -f $lat, $lon)
    $argsList = @(
        "--model", "plane",
        "--speedup", "1",
        "-I", "$instance",
        "--home", $homeArg,
        "--sysid", "$sysid",
        "--serial0", "tcp:0",
        "--sim-address=127.0.0.1",
        "--defaults", $defaults
    )
    Start-Process `
        -FilePath $planeBinary `
        -ArgumentList $argsList `
        -WorkingDirectory $dir `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $dir "arduplane.out.log") `
        -RedirectStandardError (Join-Path $dir "arduplane.err.log")
    $tcpPort = 5760 + $instance * 10
    Write-Host "started real ArduPlane SITL sysid=$sysid tcp=$tcpPort home=$homeArg"
}

for ($i = 0; $i -lt $RoverCount; $i++) {
    $dir = Join-Path $runRoot "rover_$i"
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    $sysid = 200 + $i
    $instance = 20 + $i
    $lat = $roverHome.Lat
    $lon = $roverHome.Lon
    $homeArg = ("{0:F7},{1:F7},20,0" -f $lat, $lon)
    $argsList = @(
        "--model", "rover",
        "--speedup", "1",
        "-I", "$instance",
        "--home", $homeArg,
        "--sysid", "$sysid",
        "--serial0", "tcp:0",
        "--sim-address=127.0.0.1",
        "--defaults", $defaults
    )
    Start-Process `
        -FilePath $roverBinary `
        -ArgumentList $argsList `
        -WorkingDirectory $dir `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $dir "ardurover.out.log") `
        -RedirectStandardError (Join-Path $dir "ardurover.err.log")
    $tcpPort = 5760 + $instance * 10
    Write-Host "started real ArduRover Hive SITL sysid=$sysid tcp=$tcpPort home=$homeArg"
}

Start-Sleep -Seconds 5

if (-not $NoRouter) {
    $routerArgs = @(
        "-u", (Join-Path $workspace "scripts\ardupilot_real_router.py"),
        "--copters", "$Count",
        "--planes", "$PlaneCount",
        "--rovers", "$RoverCount"
    )
    if ($VisualPlan -ne "") {
        $routerArgs += "--visual-plan"
        $routerArgs += (Resolve-Path $VisualPlan).Path
    }
    Start-Process `
        -FilePath "python" `
        -ArgumentList $routerArgs `
        -WorkingDirectory $workspace `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $runRoot "router.out.log") `
        -RedirectStandardError (Join-Path $runRoot "router.err.log")
    Write-Host "started real MAVLink router: ArduPilot TCP -> QGC UDP 14550"
}

if (-not $NoQGC -and (Test-Path $qgc)) {
    Start-Process -FilePath $qgc
    Write-Host "opened QGroundControl"
}

Write-Host "logs: $runRoot"
