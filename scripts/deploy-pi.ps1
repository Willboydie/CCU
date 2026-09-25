# Copy the CCU sources to the Raspberry Pi 5, then print one hardware gate.
#
#   .\scripts\deploy-pi.ps1 -Gate H1
#
# The script copies the tree to /home/will/smart_marker/CCU. It then prints the
# commands for that gate. Run those commands in your own SSH session.
# ssh and scp use IPv4. Password auth uses the bringup password unless you pass -Password.

param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("H1", "H2", "H2.5", "H3", "H4", "H5", "H6", "H7", "H8", "H9", "H10")]
    [string]$Gate,
    [string]$Target = "will@192.168.4.2",
    [string]$Password = "kippen",
    [string]$RemoteDir = "/home/will/smart_marker/CCU"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$copyItems = @(
    "CMakeLists.txt",
    "ccu_ports.cfg",
    "ccu_survey.cfg",
    "ccu",
    "tests",
    "lib",
    "docs",
    "scripts"
)

$sshBase = @(
    "-4",
    "-o", "ConnectTimeout=8",
    "-o", "LogLevel=ERROR",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "PreferredAuthentications=password",
    "-o", "PubkeyAuthentication=no"
)

$askPassFile = Join-Path $env:TEMP ("ccu-deploy-askpass-{0}.cmd" -f $PID)
Set-Content -Path $askPassFile -Value "@echo off`r`necho $Password" -Encoding ascii
$env:SSH_ASKPASS = $askPassFile
$env:SSH_ASKPASS_REQUIRE = "force"
if (-not $env:DISPLAY) { $env:DISPLAY = "required" }

function Get-GatePrompt([string]$name) {
    $build = @(
        "ssh -4 $Target",
        "cd ~/smart_marker/CCU",
        "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release",
        "cmake --build build --target ccu",
        "./build/ccu ccu_ports.cfg"
    )
    $setup = $null
    $watch = $null
    $pcCmd = $null
    switch ($name) {
        "H1" {
            $setup = "sudo apt update && sudo apt install -y build-essential cmake ninja-build git pkg-config libboost-dev libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev"
            $watch = "Process stays up. Each port opens. Log shows wifi listening 0.0.0.0:8765 and wifi beacon 0.0.0.0:8766."
        }
        "H2" {
            $build = @(
                "ssh -4 $Target",
                "cd ~/smart_marker/CCU",
                "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release",
                "cmake --build build --target ccu_h2",
                "./build/ccu_h2 ccu_ports.cfg"
            )
            $watch = "Hold the base still for the rate check, then flick it once and hold. Pass: stage lines near 10 Hz, roll pitch and IMU yaw stay 0, and the flick is spread across samples. Last line is H2 PASS or H2 FAIL."
        }
        "H2.5" {
            $build = @(
                "ssh -4 $Target",
                "cd ~/smart_marker/CCU",
                "python3 scripts/h2-5.py"
            )
            $pcCmd = "py -3 scripts/h2-5.py"
            $watch = "One Wi-Fi dongle is on the Pi and the other is on this PC at COM9. The tablet USB-C is not in a dongle. Start the Pi script, then the PC script. Either may start first. Both last lines are H2.5 PASS."
        }
        "H3" {
            $watch = "Tablet and Pi are both joined to willys_little_chicken. The app is installed over USB from this PC. The connection dialog lists Smart Marker, session idle, no telemetry. Do not tap Start session. The Pi log shows device_id lines and no telemetry lines."
        }
        "H4" {
            $watch = "Tap Start session. PAIRING_ACCEPTED. Telemetry at 10 Hz. Pose screen still says Set Origin. originIsSet is false. moved is false. Pose zeros."
        }
        "H5" {
            $watch = "Point the device due north. Tap Set Origin. Next telemetry has originIsSet. Pose screen shows Pose Identified. Wedge sits on (0, 0) and points up."
        }
        "H6" {
            $watch = "Turn the base clockwise about 90 degrees. Wedge turns toward +east. effectiveYaw near pi/2. Close the tablet app, or leave the phone hotspot, for 6 s. DEVICE_ID resumes. Origin stays. Open the app on the phone hotspot and Join session. Wedge matches the current base."
        }
        "H7" {
            $watch = "End session, then start again. Origin is clear. Wedge hidden. Set Origin is required again."
        }
        "H8" {
            $watch = "Survey one in-wedge vertex. Lidar is commanded. One SURVEY_RESULT with surveyFinished. Tablet marks the vertex surveyed. An out-of-wedge vertex returns OUT_OF_FOV and the lidar is not moved for it."
        }
        "H9" {
            $watch = "Project that surveyed vertex. PROJECTION_READY lists it. Laser draws the naive corner. Pause freezes it, Play resumes, End kills the beam."
        }
        "H10" {
            $watch = "Cold start. H3 through H6, then survey two vertices brought into the wedge by turning the base, then project both."
        }
    }
    [pscustomobject]@{
        Setup = $setup
        Build = $build
        Watch = $watch
        Pc = $pcCmd
    }
}

try {
    foreach ($item in $copyItems) {
        $local = Join-Path $repoRoot $item
        if (-not (Test-Path -LiteralPath $local)) {
            throw "Missing $local"
        }
    }

    Write-Host "Copying CCU sources to ${Target}:${RemoteDir}"
    & ssh @sshBase $Target "mkdir -p '$RemoteDir' && chmod -R u+rwX '$RemoteDir'"
    if ($LASTEXITCODE -ne 0) {
        throw "ssh to $Target failed (exit $LASTEXITCODE)"
    }

    foreach ($item in $copyItems) {
        $local = Join-Path $repoRoot $item
        & scp @sshBase -r -- $local "${Target}:${RemoteDir}/"
        if ($LASTEXITCODE -ne 0) {
            throw "scp $item failed (exit $LASTEXITCODE)"
        }
    }

    $prompt = Get-GatePrompt $Gate
    Write-Host ""
    Write-Host "Copied. $Gate is next. In an SSH session, run:"
    Write-Host ""
    $lines = @($prompt.Build)
    if ($prompt.Setup) {
        $head = @($lines[0], $lines[1], $prompt.Setup)
        if ($lines.Length -gt 2) {
            $lines = $head + $lines[2..($lines.Length - 1)]
        } else {
            $lines = $head
        }
    }
    $step = 1
    foreach ($line in $lines) {
        Write-Host ("  {0}. {1}" -f $step, $line)
        $step++
    }
    if ($prompt.Pc) {
        Write-Host ""
        Write-Host "On this PC, while that script is running:"
        Write-Host ("  {0}" -f $prompt.Pc)
    }
    Write-Host ""
    Write-Host "Pass: $($prompt.Watch)"
    Write-Host ""
    Write-Host "Ctrl+C any ccu already running in that session before you start this one."
}
finally {
    Remove-Item $askPassFile -Force -ErrorAction SilentlyContinue
}
