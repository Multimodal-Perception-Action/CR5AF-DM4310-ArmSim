[CmdletBinding()]
param(
    [ValidateSet("teleop", "simulation")]
    [string]$Mode = "teleop"
)

$ErrorActionPreference = "Stop"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONDONTWRITEBYTECODE = "1"
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$python = Join-Path $root ".venv\Scripts\python.exe"
$pythonw = Join-Path $root ".venv\Scripts\pythonw.exe"

$ready = $false
if (Test-Path -LiteralPath $python -PathType Leaf) {
    & $python (Join-Path $root "scripts\package_self_check.py") --quick
    $ready = $LASTEXITCODE -eq 0
}
if (-not $ready -or -not (Test-Path -LiteralPath (Join-Path $root ".installed"))) {
    & (Join-Path $root "install.ps1")
}

if ($Mode -eq "teleop") {
    if (-not (Test-Path -LiteralPath $pythonw -PathType Leaf)) { $pythonw = $python }
    $panel = Join-Path $root "src\uarm_cr5_live_panel.py"
    $logDirectory = Join-Path $root "logs"
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss_fff"
    $stdout = Join-Path $logDirectory "panel_start_$stamp.out.txt"
    $stderr = Join-Path $logDirectory "panel_start_$stamp.err.txt"
    $process = Start-Process -FilePath $pythonw -ArgumentList @("`"$panel`"") -WorkingDirectory $root -PassThru -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    if ($process.WaitForExit(2000) -and $process.ExitCode -ne 0) {
        throw "Panel startup failed. See $stderr"
    }
    Write-Host "Teleoperation panel started. Opening the panel alone does not enable or move hardware."
} else {
    $robotHost = "192.168.201.1"
    $leaderPort = "auto"
    $connectionFile = Join-Path $root "config\device_connection.json"
    if (Test-Path -LiteralPath $connectionFile) {
        $connection = Get-Content -LiteralPath $connectionFile -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($connection.robot_host) { $robotHost = [string]$connection.robot_host }
        if ($connection.leader_port) { $leaderPort = [string]$connection.leader_port }
    }
    & $python (Join-Path $root "src\uarm_cr5_dual_twin.py") --gui --duration 28800 --host $robotHost --port $leaderPort
    exit $LASTEXITCODE
}
