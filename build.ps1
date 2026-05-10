param(
    [switch]$Run,
    [switch]$TwoBlocks,
    [int]$Steps = 900
)

$ErrorActionPreference = "Stop"

function Find-Command($name) {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    return $null
}

$cmake = Find-Command "cmake"
$gcc = Find-Command "gcc"
$clang = Find-Command "clang"
$cl = Find-Command "cl"

$msys2Ucrt = "C:\msys64\ucrt64\bin"
$msys2Usr = "C:\msys64\usr\bin"
if (-not $gcc -and (Test-Path (Join-Path $msys2Ucrt "gcc.exe"))) {
    $env:Path = "$msys2Ucrt;$msys2Usr;$env:Path"
    $gcc = Join-Path $msys2Ucrt "gcc.exe"
}

New-Item -ItemType Directory -Force -Path .tmp | Out-Null
$env:TMP = (Resolve-Path .tmp).Path
$env:TEMP = $env:TMP

if ($cmake) {
    Write-Host "Using CMake: $cmake"
    cmake -S . -B build
    cmake --build build
    $exe = Join-Path "build" "Debug\scout_opt.exe"
    if (-not (Test-Path $exe)) {
        $exe = Join-Path "build" "scout_opt.exe"
    }
} elseif ($gcc) {
    Write-Host "Using GCC: $gcc"
    & $gcc -std=c11 -Wall -Wextra -pedantic -O2 -Ic_include -o scout_opt.exe c_src/main.c c_src/scout_opt.c c_src/config_loader.c c_src/sitl_bridge.c c_src/diagnostics.c c_src/visual_export.c -lm
    $exe = ".\scout_opt.exe"
} elseif ($clang) {
    Write-Host "Using Clang: $clang"
    & $clang -std=c11 -Wall -Wextra -pedantic -O2 -Ic_include -o scout_opt.exe c_src/main.c c_src/scout_opt.c c_src/config_loader.c c_src/sitl_bridge.c c_src/diagnostics.c c_src/visual_export.c
    $exe = ".\scout_opt.exe"
} elseif ($cl) {
    Write-Host "Using MSVC cl: $cl"
    & $cl /nologo /W4 /std:c11 /Ic_include c_src\main.c c_src\scout_opt.c c_src\config_loader.c c_src\sitl_bridge.c c_src\diagnostics.c c_src\visual_export.c /Fe:scout_opt.exe
    $exe = ".\scout_opt.exe"
} else {
    Write-Error "No C toolchain found. Install one of: Visual Studio Build Tools with C++ workload, MinGW-w64/MSYS2 GCC, LLVM Clang, or CMake with a configured generator."
}

if ($Run) {
    if (-not (Test-Path $exe)) {
        Write-Error "Build finished but executable was not found: $exe"
    }
    $args = @("--steps", "$Steps")
    if ($TwoBlocks) {
        $args = @("--two-blocks") + $args
    }
    & $exe @args
}
