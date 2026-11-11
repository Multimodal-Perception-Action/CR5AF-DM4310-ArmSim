[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONDONTWRITEBYTECODE = "1"
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$python = Join-Path $root ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $python -PathType Leaf) -or
    -not (Test-Path -LiteralPath (Join-Path $root ".installed"))) {
    & (Join-Path $root "install.ps1")
}
$env:PYTHONPATH = Join-Path $root "src"
& $python -m unittest discover -s (Join-Path $root "tests") -p "test_*.py" -v
exit $LASTEXITCODE
