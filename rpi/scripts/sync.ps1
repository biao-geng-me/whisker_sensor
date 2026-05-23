#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Copy the Pi-relevant subtrees from this repo to the Raspberry Pi over scp.

.DESCRIPTION
    Pushes rpi/ and hx711_array/ as full subdirectories under -Dest on the Pi.
    Uses scp (OpenSSH, built into Windows 10+) so no extra tooling is needed.
    Each invocation re-copies everything — fine for these small folders.

    Run from anywhere; paths are resolved relative to this script's location.

.PARAMETER PiHost
    user@host (e.g. pi@192.168.1.42 or pi@raspberrypi.local).

.PARAMETER Dest
    Remote destination directory. Created if missing. Default: ~/whisker_sensor

.PARAMETER Port
    Optional SSH port (default 22).

.EXAMPLE
    .\sync.ps1 -PiHost pi@192.168.1.42
    .\sync.ps1 -PiHost pi@raspberrypi.local -Dest '~/work/whisker' -Port 2222
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$PiHost,
    [string]$Dest = '~/whisker_sensor',
    [int]$Port = 22
)

$ErrorActionPreference = 'Stop'

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot  = Resolve-Path (Join-Path $ScriptDir '..\..')

$RpiDir   = Join-Path $RepoRoot 'rpi'
$Hx711Dir = Join-Path $RepoRoot 'hx711_array'

foreach ($d in @($RpiDir, $Hx711Dir)) {
    if (-not (Test-Path $d)) { throw "Source not found: $d" }
}

Write-Host "Ensuring remote dirs exist: $Dest, $Dest/hx711_array"
& ssh -p $Port $PiHost "mkdir -p $Dest/hx711_array"
if ($LASTEXITCODE -ne 0) { throw "ssh mkdir failed (exit $LASTEXITCODE)" }

# Whole rpi/ tree (no excludes needed in current layout)
& scp -P $Port -r $RpiDir ('{0}:{1}/' -f $PiHost, $Dest)
if ($LASTEXITCODE -ne 0) { throw "scp rpi/ failed (exit $LASTEXITCODE)" }

# From hx711_array/, only push the Python files (skip *.ino — Arduino source)
$PyFiles = Get-ChildItem -Path $Hx711Dir -File -Filter '*.py'
if ($PyFiles.Count -eq 0) {
    Write-Host "No .py files in hx711_array/ — skipping"
} else {
    $RemoteHx = '{0}:{1}/hx711_array/' -f $PiHost, $Dest
    & scp -P $Port @($PyFiles.FullName) $RemoteHx
    if ($LASTEXITCODE -ne 0) { throw "scp hx711_array/*.py failed (exit $LASTEXITCODE)" }
}

Write-Host "Done. On the Pi: cd $Dest && python3 -u rpi/daq_bridge.py --port /dev/ttyACM0 --bind 0.0.0.0:5555"
