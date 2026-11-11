[CmdletBinding()]
param()

$ErrorActionPreference = "Continue"
$env:PYTHONIOENCODING = "utf-8"
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$python = Join-Path $root ".venv\Scripts\python.exe"

Write-Host "=== Serial ports (enumeration only; no writes) ===" -ForegroundColor Cyan
if (Test-Path -LiteralPath $python -PathType Leaf) {
    & $python -c "from serial.tools import list_ports; [print(f'{p.device} | {p.description} | VID={p.vid} PID={p.pid} | SN={p.serial_number}') for p in list_ports.comports()]"
} else {
    Write-Host "Run the one-click installer first."
}

Write-Host "`n=== CR5/gateway TCP ports (connectivity probe only) ===" -ForegroundColor Cyan
$robotHost = "192.168.201.1"
$connectionFile = Join-Path $root "config\device_connection.json"
if (Test-Path -LiteralPath $connectionFile) {
    $connection = Get-Content -LiteralPath $connectionFile -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($connection.robot_host) { $robotHost = [string]$connection.robot_host }
}
foreach ($port in 29999, 30004, 60000) {
    $result = Test-NetConnection -ComputerName $robotHost -Port $port -InformationLevel Detailed -WarningAction SilentlyContinue
    "{0}:{1} -> {2}" -f $robotHost, $port, $result.TcpTestSucceeded
}

Write-Host "`nNo robot, gripper, or motor action command was sent."
