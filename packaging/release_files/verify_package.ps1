[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONDONTWRITEBYTECODE = "1"
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$python = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    & (Join-Path $root "install.ps1")
}
& $python (Join-Path $root "scripts\verify_integrity.py") $root
exit $LASTEXITCODE
