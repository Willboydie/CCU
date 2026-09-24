# H2.4. Idle session_poll, with paired injected, produces DEVICE_ID for the tablet parser.
#
#   .\scripts\h2-4-idle.ps1
#
# Last line is H2.4 PASS or H2.4 FAIL. No dongle and no Pi.

$ErrorActionPreference = "Stop"

function Set-JavaHome {
    $candidates = @()
    if ($env:JAVA_HOME) { $candidates += $env:JAVA_HOME }
    $candidates += @(
        "$env:ProgramFiles\Android\Android Studio\jbr",
        "$env:LOCALAPPDATA\Programs\Android\Android Studio\jbr"
    )
    foreach ($jdkHome in $candidates) {
        $java = Join-Path $jdkHome "bin\java.exe"
        if (-not (Test-Path -LiteralPath $java)) { continue }
        $env:JAVA_HOME = $jdkHome
        $env:Path = "$(Join-Path $jdkHome 'bin');$env:Path"
        return
    }
    throw "java was not found. Set JAVA_HOME to a JDK."
}

Set-JavaHome

$repoRoot = Split-Path -Parent $PSScriptRoot
$frontend = Join-Path (Split-Path -Parent $repoRoot) "SmartMarkerFrontend"
$build = Join-Path $repoRoot "build_ccu"
$markerDir = Join-Path $frontend "app\build"
$marker = Join-Path $markerDir "h2_4_exe.txt"

cmake -S $repoRoot -B $build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

cmake --build $build --target ccu_h2_4 --config Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$exe = Join-Path $build "Release\ccu_h2_4.exe"
if (-not (Test-Path -LiteralPath $exe)) {
    Write-Host "H2.4 FAIL"
    Write-Host "missing $exe"
    exit 1
}

New-Item -ItemType Directory -Force -Path $markerDir | Out-Null
Set-Content -LiteralPath $marker -Value $exe -Encoding ascii -NoNewline

Push-Location $frontend
try {
    & .\gradlew.bat :app:testDebugUnitTest --tests com.example.smartmarkerfrontend.device.H2IdleSessionPipeTest "-Pccu.h2_4.file=build/h2_4_exe.txt"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "H2.4 FAIL"
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
    Remove-Item -LiteralPath $marker -Force -ErrorAction SilentlyContinue
}

$xmlPath = Join-Path $frontend "app\build\test-results\testDebugUnitTest\TEST-com.example.smartmarkerfrontend.device.H2IdleSessionPipeTest.xml"
if (-not (Test-Path -LiteralPath $xmlPath)) {
    Write-Host "H2.4 FAIL"
    Write-Host "missing $xmlPath"
    exit 1
}

[xml]$report = Get-Content -LiteralPath $xmlPath
$suite = $report.testsuite
if ($suite.tests -ne "1" -or $suite.failures -ne "0" -or $suite.errors -ne "0" -or $suite.skipped -ne "0") {
    Write-Host "H2.4 FAIL"
    Write-Host ("tests={0} failures={1} errors={2} skipped={3}" -f $suite.tests, $suite.failures, $suite.errors, $suite.skipped)
    exit 1
}

Write-Host "H2.4 PASS"
