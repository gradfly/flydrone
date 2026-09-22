# FlyDrones PowerShell Launcher
# Usage: .\flydrones.ps1 bench  OR  .\flydrones.ps1 demo --live
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Args
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

if (-not $env:PYTHONPATH) {
    $env:PYTHONPATH = ""
}
$env:PYTHONPATH = (Join-Path $ScriptDir "src") + ";" + $env:PYTHONPATH
Write-Host "[FlyDrones] PYTHONPATH prepended: src/" -ForegroundColor Cyan

if ($Args.Count -eq 0) {
    Write-Host @"
Usage:
  .\flydrones.ps1 demo --live          Sim room + scripted hand, live dashboard
  .\flydrones.ps1 demo --record x.gif  Save demo GIF
  .\flydrones.ps1 inspect              Poke every input group
  .\flydrones.ps1 swarm --live         One connectome, three pilots
  .\flydrones.ps1 bench                Benchmark speed
  .\flydrones.ps1 fly --drone tello ...  Control a real drone
"@ -ForegroundColor Yellow
    exit 0
}

& python -m flydrones @Args
