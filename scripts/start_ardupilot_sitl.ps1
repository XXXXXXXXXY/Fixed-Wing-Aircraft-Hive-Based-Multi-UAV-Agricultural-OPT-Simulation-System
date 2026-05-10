param(
    [int]$Count = 8,
    [string]$ArduPilotRoot = "ardupilot",
    [string]$Vehicle = "ArduCopter",
    [string]$Frame = "quad",
    [string]$Location = "CMAC",
    [string]$Defaults = "configs\sitl_qgc_relaxed.parm",
    [int]$BridgeBasePort = 14600,
    [switch]$Wipe,
    [switch]$NoBridge,
    [switch]$WithMavProxy,
    [switch]$NoRebuild
)

$ErrorActionPreference = "Stop"

$currentPath = $env:Path
[System.Environment]::SetEnvironmentVariable("PATH", $null, "Process")
$env:Path = $currentPath

$msysBin = "C:\msys64\usr\bin"
if (Test-Path $msysBin) {
    $env:Path = "$env:Path;$msysBin"
}

if (-not $NoRebuild) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build_ardupilot_sitl.ps1") -ArduPilotRoot $ArduPilotRoot
    if ($LASTEXITCODE -ne 0) {
        Write-Error "ArduPilot build failed."
    }
}

$binary = Join-Path $ArduPilotRoot "build\sitl\bin\arducopter.exe"
if (-not (Test-Path $binary)) {
    Write-Error "No ArduCopter SITL binary found: $binary. Run .\scripts\build_ardupilot_sitl.ps1 first."
}

function Get-SitlLocation($Name) {
    $locations = Join-Path $ArduPilotRoot "Tools\autotest\locations.txt"
    $line = Get-Content $locations | Where-Object { $_ -match "^\s*$Name\s*=" } | Select-Object -First 1
    if (-not $line) {
        Write-Error "Location not found in $locations : $Name"
    }
    $value = ($line -split "=", 2)[1]
    $value = ($value -split "#", 2)[0].Trim()
    $parts = $value -split ","
    return @{
        Lat = [double]$parts[0]
        Lon = [double]$parts[1]
        Alt = [double]$parts[2]
        Heading = [double]$parts[3]
    }
}

function New-OffsetHome($Base, [int]$Index) {
    $bearingDeg = 90.0
    $distanceM = 10.0 * $Index
    $bearing = $bearingDeg * [Math]::PI / 180.0
    $north = [Math]::Cos($bearing) * $distanceM
    $east = [Math]::Sin($bearing) * $distanceM
    $lat = $Base.Lat + ($north / 111111.0)
    $lon = $Base.Lon + ($east / (111111.0 * [Math]::Cos($Base.Lat * [Math]::PI / 180.0)))
    return "{0},{1},{2},{3}" -f $lat.ToString("F7"), $lon.ToString("F7"), $Base.Alt.ToString("F1"), $Base.Heading.ToString("F0")
}

$baseLocation = Get-SitlLocation $Location
$runRoot = Join-Path (Resolve-Path ".").Path ".tmp\sitl"
New-Item -ItemType Directory -Force -Path $runRoot | Out-Null
$defaultsPath = $null
if ($Defaults) {
    $defaultsPath = (Resolve-Path $Defaults).Path
}

Write-Host "Starting $Count ArduPilot SITL vehicles for QGroundControl..."
$masterArgs = @()
for ($i = 0; $i -lt $Count; $i++) {
    $instanceDir = Join-Path $runRoot ("vehicle_" + $i)
    New-Item -ItemType Directory -Force -Path $instanceDir | Out-Null
    if ($Wipe) {
        Remove-Item -LiteralPath (Join-Path $instanceDir "eeprom.bin") -Force -ErrorAction SilentlyContinue
    }
    $homeArg = New-OffsetHome $baseLocation $i
    $sysid = $i + 1
    $argsList = @(
        "--model", $Frame,
        "--speedup", "1",
        "-I", "$i",
        "--home", $homeArg,
        "--sysid", "$sysid",
        "--serial0", "tcp:0",
        "--serial1", ("udpclient:127.0.0.1:" + ($BridgeBasePort + $i)),
        "--sim-address=127.0.0.1"
    )
    if ($defaultsPath) {
        $argsList += "--defaults"
        $argsList += $defaultsPath
    }
    $stdoutLog = Join-Path $instanceDir "arducopter.out.log"
    $stderrLog = Join-Path $instanceDir "arducopter.err.log"
    Remove-Item -LiteralPath $stdoutLog, $stderrLog -Force -ErrorAction SilentlyContinue
    Start-Process `
        -FilePath (Resolve-Path $binary).Path `
        -ArgumentList $argsList `
        -WorkingDirectory $instanceDir `
        -WindowStyle Hidden `
        -RedirectStandardOutput $stdoutLog `
        -RedirectStandardError $stderrLog
    $masterArgs += "--master"
    $masterArgs += ("tcp:127.0.0.1:" + (5760 + $i * 10))
    Write-Host "  vehicle_$i sysid=$sysid home=$homeArg"
}

if (-not $NoBridge) {
    Start-Sleep -Seconds 2
    $bridgeArgs = @("-m", "MAVProxy.mavproxy") + $masterArgs + @(
        "--out", "udp:127.0.0.1:14550",
        "--source-system", "255",
        "--no-console",
        "--non-interactive",
        "--daemon"
    )
    Start-Process `
        -FilePath "python" `
        -ArgumentList $bridgeArgs `
        -WorkingDirectory $runRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $runRoot "mavproxy.out.log") `
        -RedirectStandardError (Join-Path $runRoot "mavproxy.err.log")
    Write-Host "MAVProxy bridge started: SITL TCP masters -> QGroundControl UDP 14550"
}

Write-Host "Open QGroundControl and wait for UDP 14550 vehicles to appear."
