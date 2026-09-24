# H2.3. Tablet open emits DONGLE_RESET, then a DEVICE_ID after a status frame.
#
#   .\scripts\h2-3-tablet-open.ps1
#
# Last line is H2.3 PASS or H2.3 FAIL. No dongle and no Pi.

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

Push-Location $frontend
try {
    & .\gradlew.bat :app:testDebugUnitTest --tests com.example.smartmarkerfrontend.device.H2TabletOpenTest
    if ($LASTEXITCODE -ne 0) {
        Write-Host "H2.3 FAIL"
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}

$xmlPath = Join-Path $frontend "app\build\test-results\testDebugUnitTest\TEST-com.example.smartmarkerfrontend.device.H2TabletOpenTest.xml"
if (-not (Test-Path -LiteralPath $xmlPath)) {
    Write-Host "H2.3 FAIL"
    Write-Host "missing $xmlPath"
    exit 1
}

[xml]$report = Get-Content -LiteralPath $xmlPath
$suite = $report.testsuite
if ($suite.tests -ne "1" -or $suite.failures -ne "0" -or $suite.errors -ne "0" -or $suite.skipped -ne "0") {
    Write-Host "H2.3 FAIL"
    Write-Host ("tests={0} failures={1} errors={2} skipped={3}" -f $suite.tests, $suite.failures, $suite.errors, $suite.skipped)
    exit 1
}

Write-Host "H2.3 PASS"
