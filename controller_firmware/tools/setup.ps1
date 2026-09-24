# One-shot Windows setup: installs what's missing, builds the simulator and
# tests, runs them, and builds the firmware if the ARM toolchain is present.
# Run via tools\setup.bat (double-click) or:
#   powershell -ExecutionPolicy Bypass -File tools\setup.ps1
param([switch]$NoInstall)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $PSScriptRoot          # controller_firmware
Set-Location $root

function Step($msg) { Write-Host "`n==> $msg" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "    $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "    $msg" -ForegroundColor Yellow }
function Have($cmd) { [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }

function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" +
                [Environment]::GetEnvironmentVariable("Path", "User") + ";" + $env:Path
}

function Add-Path($dir) {
    if ($dir -and (Test-Path $dir) -and -not ($env:Path -split ";" -contains $dir)) { $env:Path = "$dir;$env:Path" }
}

function Winget-Install($id, $what) {
    if ($NoInstall) { Warn "$what missing (skipping install: -NoInstall)"; return }
    if (-not (Have winget)) { Warn "$what missing and winget is not available - install it manually"; return }
    Write-Host "    installing $what ($id)..."
    winget install --id $id -e --silent --accept-package-agreements --accept-source-agreements | Out-Host
    Refresh-Path
}

# ---- STM32CubeCLT (cmake, ninja, arm gcc, programmer) if installed ----
Step "Looking for STM32CubeCLT"
$clt = $null
if (Test-Path "C:\ST") {
    $clt = Get-ChildItem "C:\ST" -Directory -Filter "STM32CubeCLT*" -ErrorAction SilentlyContinue |
           Sort-Object Name -Descending | Select-Object -First 1
}
if ($clt) {
    foreach ($sub in "CMake\bin", "Ninja\bin", "GNU-tools-for-STM32\bin", "STM32CubeProgrammer\bin") {
        Add-Path (Join-Path $clt.FullName $sub)
    }
    Ok "found $($clt.FullName)"
} else {
    Warn "not found - firmware build/flash needs it: https://www.st.com/en/development-tools/stm32cubeclt.html"
}

# ---- Python + dashboard packages ----
Step "Python"
$py = $null
foreach ($c in "py", "python") {
    if (Have $c) {
        try { & $c -c "import sys; assert sys.version_info >= (3, 8)" 2>$null; if ($LASTEXITCODE -eq 0) { $py = $c; break } } catch {}
    }
}
if (-not $py) {
    Winget-Install "Python.Python.3.12" "Python 3.12"
    foreach ($c in "py", "python") { if (Have $c) { $py = $c; break } }
}
if (-not $py) { throw "Python not available. Install it from https://www.python.org and re-run." }
Ok (& $py --version)
& $py -m pip install --quiet --disable-pip-version-check -r (Join-Path $root "tools\dashboard\requirements.txt") pytest
if ($LASTEXITCODE -eq 0) { Ok "tornado, pyserial, pytest installed" } else { Warn "pip install failed - see output above" }

# ---- CMake / Ninja ----
Step "CMake and Ninja"
if (-not (Have cmake)) { Winget-Install "Kitware.CMake" "CMake" }
if (-not (Have ninja)) { Winget-Install "Ninja-build.Ninja" "Ninja" }
if ((Have cmake) -and (Have ninja)) { Ok "cmake $((cmake --version)[0] -replace 'cmake version ','') , ninja $(ninja --version)" }

# ---- host C compiler for the simulator ----
Step "Host C compiler"
function Find-WinLibs {
    $pkgs = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages"
    Get-ChildItem $pkgs -Directory -Filter "BrechtSanders.WinLibs*" -ErrorAction SilentlyContinue |
        ForEach-Object { Join-Path $_.FullName "mingw64\bin" } | Where-Object { Test-Path "$_\gcc.exe" } | Select-Object -First 1
}
if (-not (Have gcc) -and -not (Have cl)) {
    Add-Path (Find-WinLibs)
    if (-not (Have gcc)) {
        Winget-Install "BrechtSanders.WinLibs.POSIX.UCRT" "MinGW-w64 GCC (WinLibs)"
        Add-Path (Find-WinLibs)
    }
}
$compiler = $null
if (Have gcc) { $compiler = "gcc"; $env:CC = "gcc" } elseif (Have cl) { $compiler = "cl"; $env:CC = "cl" }
if ($compiler) { Ok "using $compiler" } else { Warn "no C compiler - the prebuilt build-sim\foc_sim.exe (if present) still runs" }

# ---- simulator + tests ----
$simOk = $false
if ($compiler -and (Have cmake) -and (Have ninja)) {
    Step "Building simulator and tests"
    if (Test-Path build-sim\CMakeCache.txt) { Remove-Item build-sim\CMakeCache.txt, build-sim\CMakeFiles -Recurse -Force }
    cmake --preset sim | Out-Host
    if ($LASTEXITCODE -eq 0) { cmake --build build-sim | Out-Host }
    if ($LASTEXITCODE -eq 0) {
        Step "Running unit + SIL tests"
        ctest --preset sim | Out-Host
        $simOk = ($LASTEXITCODE -eq 0)
    }
    if ($simOk) { Ok "all C tests passed" } else { Warn "C build or tests failed - see output above" }
}

if (Test-Path build-sim\foc_sim.exe) {
    Step "Running dashboard tests"
    $env:FOC_SIM = (Resolve-Path build-sim\foc_sim.exe).Path
    & $py -m pytest -q (Join-Path $root "tools\dashboard\tests") | Out-Host
    if ($LASTEXITCODE -eq 0) { Ok "dashboard + end-to-end tests passed" } else { Warn "dashboard tests failed" }
}

# ---- firmware ----
if ((Have arm-none-eabi-gcc) -and (Have cmake) -and (Have ninja)) {
    Step "Building firmware"
    cmake --preset default | Out-Host
    if ($LASTEXITCODE -eq 0) { cmake --build build | Out-Host }
    if ($LASTEXITCODE -eq 0) { Ok "build\foc_g474.elf ready - flash with: cmake --build build --target flash" }
} else {
    Warn "arm-none-eabi-gcc not found - skipping firmware build (install STM32CubeCLT)"
}

Step "Done"
if (Test-Path build-sim\foc_sim.exe) {
    Ok "start the simulated motor + dashboard with tools\dashboard.bat"
}
