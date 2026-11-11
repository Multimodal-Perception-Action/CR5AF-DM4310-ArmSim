[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONDONTWRITEBYTECODE = "1"
$root = [IO.Path]::GetFullPath($PSScriptRoot)
$venvPython = Join-Path $root ".venv\Scripts\python.exe"
$localPython = Join-Path $root ".python\python.exe"
$installer = Join-Path $root "runtime\python-3.13.9-amd64.exe"
$requirements = Join-Path $root "requirements-runtime.txt"
$wheels = Join-Path $root "wheels"

function Test-CompatiblePython([string]$pythonPath) {
    if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) { return $false }
    try {
        & $pythonPath -c "import struct,sys,tkinter,venv; raise SystemExit(0 if sys.version_info[:2] == (3,13) and struct.calcsize('P') == 8 else 1)" 2>$null
        return $LASTEXITCODE -eq 0
    } catch { return $false }
}

function Find-CompatiblePython {
    $candidates = @($localPython)
    try { $candidates += (& py -3.13 -c "import sys; print(sys.executable)" 2>$null) } catch {}
    # Python can be installed without the optional py launcher.
    foreach ($commandName in "python.exe", "python3.exe") {
        $command = Get-Command $commandName -CommandType Application -ErrorAction SilentlyContinue
        if ($command -and $command.Source -notlike "*\Microsoft\WindowsApps\*") {
            $candidates += $command.Source
        }
    }
    foreach ($key in @(
        "HKCU:\Software\Python\PythonCore\3.13\InstallPath",
        "HKLM:\Software\Python\PythonCore\3.13\InstallPath"
    )) {
        if (Test-Path $key) {
            $item = Get-Item $key
            $executable = $item.GetValue("ExecutablePath")
            if ($executable) { $candidates += $executable }
            $directory = $item.GetValue("")
            if ($directory) { $candidates += Join-Path $directory "python.exe" }
        }
    }
    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if ($candidate -and (Test-CompatiblePython $candidate.Trim())) { return $candidate.Trim() }
    }
    return $null
}

Write-Host "[1/4] Checking Python 3.13 x64..." -ForegroundColor Cyan
$basePython = Find-CompatiblePython
if (-not $basePython) {
    if (Test-Path -LiteralPath $installer -PathType Leaf) {
        $expectedHash = "200DDFF856BBFF949D2CC1BE42E8807C07538ABD6B6966D5113A094CF628C5C5"
        if ((Get-FileHash -LiteralPath $installer -Algorithm SHA256).Hash -ne $expectedHash) {
            throw "Bundled Python installer checksum does not match. Extract a fresh package."
        }
        Write-Host "Compatible Python not found; installing the bundled runtime into .python..." -ForegroundColor Yellow
        $pythonDir = Join-Path $root ".python"
        $arguments = (
            "/quiet InstallAllUsers=0 TargetDir=`"$pythonDir`" " +
            "AssociateFiles=0 Shortcuts=0 Include_launcher=0 PrependPath=0 " +
            "Include_test=0 Include_doc=0 Include_debug=0 Include_symbols=0 " +
            "Include_dev=0 Include_tcltk=1 Include_pip=1 Include_exe=1 " +
            "Include_lib=1 Include_tools=1"
        )
        $process = Start-Process -FilePath $installer -ArgumentList $arguments -WindowStyle Hidden -Wait -PassThru
        $basePython = Find-CompatiblePython
        if ($process.ExitCode -notin @(0, 3010) -or -not $basePython) {
            throw "Bundled Python installation failed with exit code $($process.ExitCode)."
        }
    } else {
        throw "Python 3.13 x64 was not found and the bundled installer is missing."
    }
}

if (-not (Test-Path -LiteralPath (Join-Path $root ".installed"))) {
    & $basePython (Join-Path $root "scripts\verify_integrity.py") $root
    if ($LASTEXITCODE -ne 0) { throw "Initial package checksum verification failed." }
}

Write-Host "[2/4] Creating an isolated virtual environment..." -ForegroundColor Cyan
if (-not (Test-CompatiblePython $venvPython)) {
    & $basePython -m venv (Join-Path $root ".venv")
    if ($LASTEXITCODE -ne 0) { throw "Virtual environment creation failed." }
}
& $venvPython -m ensurepip --upgrade
if ($LASTEXITCODE -ne 0) { throw "Virtual environment pip bootstrap failed." }

Write-Host "[3/4] Installing dependencies from bundled wheels..." -ForegroundColor Cyan
if (-not (Test-Path -LiteralPath $wheels -PathType Container)) {
    throw "Bundled wheel directory does not exist: $wheels"
}
& $venvPython -m pip install --disable-pip-version-check --no-index --find-links $wheels -r $requirements
if ($LASTEXITCODE -ne 0) { throw "Python dependency installation failed." }
& $venvPython -m pip check
if ($LASTEXITCODE -ne 0) { throw "Installed dependency compatibility check failed." }

Write-Host "[4/4] Running offline import and asset checks..." -ForegroundColor Cyan
& $venvPython (Join-Path $root "scripts\package_self_check.py")
if ($LASTEXITCODE -ne 0) { throw "Package self-check failed." }

Set-Content -LiteralPath (Join-Path $root ".installed") -Value (Get-Date -Format o) -Encoding UTF8
Write-Host "Installation complete. Run the one-click teleoperation launcher." -ForegroundColor Green
