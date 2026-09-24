# List USB serial ports on the Raspberry Pi 5, and show what changed since the last run.
#
#   .\scripts\pi-serial-ports.ps1
#   .\scripts\pi-serial-ports.ps1 -Watch
#
# Unplug one microcontroller and run the script again. The missing row is that board.
# /dev/ttyACM numbers can change after a replug. The USB socket and the board serial do not.
# ssh prompts for the Pi password unless you already have a key, or you pass -Password.

param(
    [string]$Target = "will@truelinepi5.local",
    [string]$Password,
    [switch]$Watch,
    [int]$IntervalSeconds = 1
)

$ErrorActionPreference = "Stop"

$remoteSh = @'
#!/bin/bash
mode=${1:-snapshot}
interval=${2:-1}
export LC_ALL=C

list_ports() {
    shopt -s nullglob
    local link tty name serial friendly path base ctrl port socket p
    for link in /dev/serial/by-id/*; do
        tty=$(readlink -f "$link")
        name=$(basename "$link")
        serial=$(udevadm info -q property -n "$tty" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p' | head -n 1)
        friendly=${name#usb-}
        friendly=${friendly%-if[0-9][0-9]}
        if [ -n "$serial" ]; then
            friendly=${friendly%_"$serial"}
        fi
        friendly=${friendly//_/ }
        path=""
        for p in /dev/serial/by-path/*; do
            base=$(basename "$p")
            case "$base" in
                *usbv2*) continue ;;
            esac
            if [ "$(readlink -f "$p")" = "$tty" ]; then
                path=$base
                break
            fi
        done
        ctrl=$(printf '%s\n' "$path" | sed -n 's/.*xhci-hcd\.\([0-9][0-9]*\).*/\1/p')
        port=$(printf '%s\n' "$path" | sed -n 's/.*usb-0:\([0-9][0-9]*\):.*/\1/p')
        if [ -n "$ctrl" ] && [ -n "$port" ]; then
            socket="xhci-hcd.${ctrl} port ${port}"
        else
            socket=${path:-unknown}
        fi
        printf 'tty=%s\tsocket=%s\tdevice=%s\tserial=%s\tpath=%s\n' \
            "$(basename "$tty")" "$socket" "$friendly" "$serial" "$path"
    done | sort -t "$(printf '\t')" -k2,2 -k4,4
}

print_tsv() {
    local data=$1
    if [ -z "$data" ]; then
        echo "  (none)"
        return
    fi
    printf '%s\n' "$data" | while IFS="$(printf '\t')" read -r a b c d e; do
        [ -z "$a" ] && continue
        tty=${a#tty=}
        socket=${b#socket=}
        device=${c#device=}
        serial=${d#serial=}
        printf '  /dev/%-12s  %-22s  %-40s  %s\n' "$tty" "$socket" "$device" "$serial"
    done
}

if [ "$mode" = "watch" ]; then
    echo "Watching USB serial ports on $(hostname). Plug or unplug a board. Ctrl+C to stop."
    echo "PORT              USB SOCKET              DEVICE                                    SERIAL"
    prev=""
    while true; do
        cur=$(list_ports)
        if [ "$cur" != "$prev" ]; then
            echo
            echo "======== $(date '+%H:%M:%S') ========"
            if [ -n "$prev" ]; then
                removed=$(comm -23 <(printf '%s\n' "$prev") <(printf '%s\n' "$cur"))
                added=$(comm -13 <(printf '%s\n' "$prev") <(printf '%s\n' "$cur"))
                echo "REMOVED:"
                print_tsv "$removed"
                echo "ADDED:"
                print_tsv "$added"
            fi
            echo "NOW CONNECTED:"
            print_tsv "$cur"
            prev=$cur
        fi
        sleep "$interval"
    done
else
    list_ports
fi
'@

$remoteSh = $remoteSh -replace "`r", ""
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($remoteSh))

$askPassFile = $null
if ($Password) {
    $askPassFile = Join-Path $env:TEMP ("pi-serial-askpass-{0}.cmd" -f $PID)
    Set-Content -Path $askPassFile -Value "@echo off`r`necho $Password" -Encoding ascii
    $env:SSH_ASKPASS = $askPassFile
    $env:SSH_ASKPASS_REQUIRE = "force"
    if (-not $env:DISPLAY) { $env:DISPLAY = "required" }
}

function Get-SnapshotPath {
    $dir = Join-Path $env:LOCALAPPDATA "smart_marker"
    if (-not (Test-Path $dir)) {
        New-Item -ItemType Directory -Path $dir | Out-Null
    }
    $safe = ($Target -replace "[^A-Za-z0-9._-]", "_")
    Join-Path $dir ("pi-serial-ports.{0}.last.txt" -f $safe)
}

function Parse-PortLine([string]$line) {
    $map = @{}
    foreach ($part in ($line -split "`t")) {
        $eq = $part.IndexOf("=")
        if ($eq -lt 1) { continue }
        $map[$part.Substring(0, $eq)] = $part.Substring($eq + 1)
    }
    [pscustomobject]@{
        Tty    = "/dev/" + $map["tty"]
        Socket = $map["socket"]
        Device = $map["device"]
        Serial = $map["serial"]
        Path   = $map["path"]
        Raw    = $line
    }
}

function Get-RowKey($row) {
    if ($row.Serial) { return $row.Serial }
    return $row.Path
}

function Show-Ports($rows) {
    Write-Host ("{0,-16} {1,-24} {2,-42} {3}" -f "PORT", "USB SOCKET", "DEVICE", "SERIAL")
    Write-Host ("{0,-16} {1,-24} {2,-42} {3}" -f "----", "----------", "------", "------")
    if (-not $rows -or $rows.Count -eq 0) {
        Write-Host "(no USB serial devices)"
        return
    }
    foreach ($row in ($rows | Sort-Object Socket, Serial)) {
        Write-Host ("{0,-16} {1,-24} {2,-42} {3}" -f $row.Tty, $row.Socket, $row.Device, $row.Serial)
    }
}

$sshBase = @(
    "-o", "ConnectTimeout=8",
    "-o", "LogLevel=ERROR",
    "-o", "StrictHostKeyChecking=accept-new"
)

try {
    if ($Watch) {
        $remoteCommand = "echo $b64 | base64 -d | stdbuf -oL bash -s -- watch $IntervalSeconds"
        $watchArgs = @("-t") + $sshBase
        & ssh @watchArgs $Target $remoteCommand
        if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 255 -and $LASTEXITCODE -ne 130) {
            throw "ssh exited with code $LASTEXITCODE"
        }
        return
    }

    $remoteCommand = "echo $b64 | base64 -d | bash -s -- snapshot"
    $output = & ssh @sshBase $Target $remoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "ssh to $Target failed (exit $LASTEXITCODE). Check that the Pi is up, then run: ssh $Target"
    }

    $lines = @($output | Where-Object { $_ -like "tty=*" })
    $rows = @($lines | ForEach-Object { Parse-PortLine $_ })

    Write-Host "USB serial ports on $Target"
    Write-Host ""
    Show-Ports $rows
    Write-Host ""

    $snapPath = Get-SnapshotPath
    $previous = @()
    if (Test-Path $snapPath) {
        $previous = @(Get-Content -Path $snapPath | Where-Object { $_ -like "tty=*" } | ForEach-Object { Parse-PortLine $_ })
    }

    if ($previous.Count -eq 0) {
        Write-Host "Saved this list. Unplug one board and run this script again."
        Write-Host "The row that disappears is that board. The USB socket stays with the physical port."
    }
    else {
        $oldByKey = @{}
        foreach ($row in $previous) { $oldByKey[(Get-RowKey $row)] = $row }
        $newByKey = @{}
        foreach ($row in $rows) { $newByKey[(Get-RowKey $row)] = $row }

        $removed = @($previous | Where-Object { -not $newByKey.ContainsKey((Get-RowKey $_)) })
        $added = @($rows | Where-Object { -not $oldByKey.ContainsKey((Get-RowKey $_)) })
        $moved = @()
        foreach ($row in $rows) {
            $key = Get-RowKey $row
            if (-not $oldByKey.ContainsKey($key)) { continue }
            $old = $oldByKey[$key]
            if ($old.Tty -ne $row.Tty -or $old.Socket -ne $row.Socket) {
                $moved += [pscustomobject]@{ Old = $old; New = $row }
            }
        }

        if ($removed.Count -eq 0 -and $added.Count -eq 0 -and $moved.Count -eq 0) {
            Write-Host "No change since the last run."
        }
        else {
            Write-Host "Since the last run:"
            foreach ($row in $removed) {
                Write-Host ("  unplugged  {0}  serial {1}  was {2} ({3})" -f $row.Device, $row.Serial, $row.Socket, $row.Tty)
            }
            foreach ($row in $added) {
                Write-Host ("  plugged in {0}  serial {1}  at {2} ({3})" -f $row.Device, $row.Serial, $row.Socket, $row.Tty)
            }
            foreach ($item in $moved) {
                Write-Host ("  moved      {0}  serial {1}  {2} {3} -> {4} {5}" -f $item.New.Device, $item.New.Serial, $item.Old.Socket, $item.Old.Tty, $item.New.Socket, $item.New.Tty)
            }
        }
    }

    $lines | Set-Content -Path $snapPath -Encoding ascii
}
finally {
    if ($askPassFile) {
        Remove-Item $askPassFile -Force -ErrorAction SilentlyContinue
    }
}
