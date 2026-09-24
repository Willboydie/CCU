# H2.2. Build and run the injected-dongle unit test. No serial port, no tablet, no Pi.
#
#   .\scripts\h2-2-dongle.ps1
#
# Last line is H2.2 PASS or H2.2 FAIL.

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$build = Join-Path $repoRoot "build_ccu"

cmake -S $repoRoot -B $build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

cmake --build $build --target ccu_h2_2 --config Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$exe = Join-Path $build "Release\ccu_h2_2.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    Write-Host "H2.2 FAIL"
    Write-Host "missing $exe"
    exit 1
}

& $exe
if ($LASTEXITCODE -ne 0) {
    Write-Host "H2.2 FAIL"
    exit $LASTEXITCODE
}

Write-Host "H2.2 PASS"
