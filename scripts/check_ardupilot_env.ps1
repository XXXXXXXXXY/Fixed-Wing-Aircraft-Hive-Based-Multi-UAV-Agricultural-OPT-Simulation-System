param(
    [string]$ArduPilotRoot = "ardupilot"
)

$ErrorActionPreference = "Continue"

function Show-Check($Name, $Ok, $Detail) {
    if ($Ok) {
        Write-Host "[OK]   $Name - $Detail"
    } else {
        Write-Host "[MISS] $Name - $Detail"
    }
}

$simVehicle = Join-Path $ArduPilotRoot "Tools\autotest\sim_vehicle.py"
$wafLight = Join-Path $ArduPilotRoot "modules\waf\waf-light"
$mavlink = Join-Path $ArduPilotRoot "modules\mavlink"
$arducopterExe = Get-ChildItem -Path (Join-Path $ArduPilotRoot "build") -Recurse -File -Filter "arducopter.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
$msysBash = Test-Path "C:\msys64\usr\bin\bash.exe"
$wsl = Get-Command wsl -ErrorAction SilentlyContinue

$python = Get-Command python -ErrorAction SilentlyContinue
Show-Check "Python" ($null -ne $python) ($(if ($python) { & python --version } else { "python command not found" }))
Show-Check "sim_vehicle.py" (Test-Path $simVehicle) $simVehicle
Show-Check "waf submodule" (Test-Path $wafLight) $wafLight
Show-Check "mavlink submodule" ((Test-Path $mavlink) -and ((Get-ChildItem $mavlink -Force -ErrorAction SilentlyContinue | Measure-Object).Count -gt 0)) $mavlink
Show-Check "ArduCopter SITL binary" ($null -ne $arducopterExe) ($(if ($arducopterExe) { $arducopterExe.FullName } else { "not built yet" }))
Show-Check "MSYS2 bash" $msysBash "C:\msys64\usr\bin\bash.exe"
Show-Check "WSL command" ($null -ne $wsl) ($(if ($wsl) { $wsl.Source } else { "wsl.exe not found" }))

$mavproxyOk = $false
$mavproxyDetail = ""
try {
    $out = python -m MAVProxy.mavproxy --version 2>&1
    $mavproxyOk = $LASTEXITCODE -eq 0
    $mavproxyDetail = ($out | Select-Object -First 1)
} catch {
    $mavproxyDetail = $_.Exception.Message
}
Show-Check "MAVProxy Python module" $mavproxyOk $mavproxyDetail

Write-Host ""
Write-Host "If waf is missing, run:"
Write-Host "  git -C $ArduPilotRoot submodule update --init --recursive modules/waf modules/mavlink"
Write-Host "If MAVProxy reports missing wx, map/console mode needs wxPython:"
Write-Host "  python -m pip install wxPython"
Write-Host "Build on this Windows host with:"
Write-Host "  .\scripts\build_ardupilot_sitl.ps1"
