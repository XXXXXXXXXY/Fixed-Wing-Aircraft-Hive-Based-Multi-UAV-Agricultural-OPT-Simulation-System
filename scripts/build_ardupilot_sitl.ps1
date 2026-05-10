param(
    [string]$ArduPilotRoot = "ardupilot",
    [int]$Jobs = 4,
    [string[]]$Targets = @("copter")
)

$ErrorActionPreference = "Stop"

$waf = Join-Path $ArduPilotRoot "waf"
if (-not (Test-Path $waf)) {
    Write-Error "waf launcher not found: $waf"
}

$msysBash = "C:\msys64\usr\bin\bash.exe"
if (-not (Test-Path $msysBash)) {
    Write-Error "MSYS2 bash not found: $msysBash. Install MSYS2 with base-devel gcc python git."
}

$rootPath = (Resolve-Path $ArduPilotRoot).Path.Replace("\", "/")
if ($rootPath -match "^([A-Za-z]):/(.*)$") {
    $drive = $Matches[1].ToLower()
    $rest = $Matches[2]
    $rootMsys = "/$drive/$rest"
} else {
    Write-Error "Unable to convert ArduPilot path to MSYS path: $rootPath"
}

$workspace = (Resolve-Path ".").Path.Replace("\", "/")
if ($workspace -match "^([A-Za-z]):/(.*)$") {
    $workspaceMsys = "/$($Matches[1].ToLower())/$($Matches[2])"
} else {
    Write-Error "Unable to convert workspace path to MSYS path: $workspace"
}

$buildCommand = @"
export HOME=$workspaceMsys/.tmp/msys_home
export TMPDIR=$workspaceMsys/.tmp/msys_tmp
export TMP=`$TMPDIR
export TEMP=`$TMPDIR
mkdir -p "`$HOME" "`$TMPDIR"
cd "$rootMsys"
git config --global --add safe.directory "$rootMsys"
python waf configure --board sitl --disable-networking
python waf $($Targets -join ' ') -j$Jobs
"@

$env:HOME = Join-Path (Resolve-Path ".").Path ".tmp\msys_home"
$env:TMPDIR = Join-Path (Resolve-Path ".").Path ".tmp\msys_tmp"
$env:TMP = $env:TMPDIR
$env:TEMP = $env:TMPDIR
New-Item -ItemType Directory -Force -Path $env:HOME, $env:TMPDIR | Out-Null

Write-Host "Configuring ArduPilot SITL..."
Write-Host "Building ArduCopter SITL with MSYS2..."
& $msysBash -lc $buildCommand
if ($LASTEXITCODE -ne 0) {
    Write-Error "ArduPilot SITL build failed."
}
