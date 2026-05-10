param(
    [int]$Count = 8
)

$ErrorActionPreference = "Stop"

$currentPath = $env:Path
[System.Environment]::SetEnvironmentVariable("PATH", $null, "Process")
$env:Path = $currentPath

$workspace = (Resolve-Path ".").Path
$bash = "C:\msys64\usr\bin\bash.exe"
$script = Join-Path $workspace "scripts\start_sim_vehicle.sh"
$logDir = Join-Path $workspace ".tmp\sim_vehicle_logs"

if (-not (Test-Path $bash)) {
    Write-Error "MSYS2 bash not found: $bash"
}
if (-not (Test-Path $script)) {
    Write-Error "Missing start script: $script"
}

New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$old = @(Get-Process arducopter,python,bash -ErrorAction SilentlyContinue)
if ($old.Count -gt 0) {
    Stop-Process -Id ($old | Select-Object -ExpandProperty Id) -Force -ErrorAction SilentlyContinue
}

& $bash -lc "chmod +x /c/Users/XY/Desktop/Drone/scripts/start_sim_vehicle.sh /c/Users/XY/Desktop/Drone/scripts/msys_bin/pkill /c/Users/XY/Desktop/Drone/scripts/msys_bin/ritw_exec"

Start-Process `
    -FilePath $bash `
    -ArgumentList @($script.Replace("\", "/"), "$Count") `
    -WorkingDirectory $workspace `
    -WindowStyle Hidden `
    -RedirectStandardOutput (Join-Path $logDir "sim_vehicle.out.log") `
    -RedirectStandardError (Join-Path $logDir "sim_vehicle.err.log")

Write-Host "ArduPilot sim_vehicle started for $Count vehicles."
Write-Host "UDP master ports: 5760, 5770, 5780, ... ; QGC/bridge can attach through the Python bridge."
