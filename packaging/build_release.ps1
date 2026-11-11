[CmdletBinding()]
param(
    [string]$Version = "v0.1.0",
    [switch]$SkipZip
)

$ErrorActionPreference = "Stop"
$project = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$dist = [IO.Path]::GetFullPath((Join-Path $project "dist"))
$releaseName = "uarm-cr5af-dm4310-teleop-windows-x64-$Version"
$stage = [IO.Path]::GetFullPath((Join-Path $dist $releaseName))
$distPrefix = $dist.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
if (-not $stage.StartsWith($distPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw "拒绝在 dist 之外创建或清理交付目录：$stage"
}

function Copy-Directory([string]$sourceRelative, [string]$destinationRelative) {
    $source = Join-Path $project $sourceRelative
    $destination = Join-Path $stage $destinationRelative
    if (-not (Test-Path -LiteralPath $source -PathType Container)) {
        throw "源目录不存在：$source"
    }
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    Copy-Item -Path (Join-Path $source "*") -Destination $destination -Recurse -Force
}

function Copy-File([string]$sourceRelative, [string]$destinationRelative) {
    $source = Join-Path $project $sourceRelative
    $destination = Join-Path $stage $destinationRelative
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
        throw "源文件不存在：$source"
    }
    New-Item -ItemType Directory -Path (Split-Path -Parent $destination) -Force | Out-Null
    Copy-Item -LiteralPath $source -Destination $destination -Force
}

New-Item -ItemType Directory -Path $dist -Force | Out-Null
if (Test-Path -LiteralPath $stage) {
    Remove-Item -LiteralPath $stage -Recurse -Force
}
New-Item -ItemType Directory -Path $stage -Force | Out-Null

# Runtime source, calibration, documents, tests, and launch support.
New-Item -ItemType Directory -Path (Join-Path $stage "src") -Force | Out-Null
Get-ChildItem -LiteralPath (Join-Path $project "src") -Filter "*.py" -File |
    Copy-Item -Destination (Join-Path $stage "src") -Force
Copy-Directory "config" "config"
Copy-Directory "docs" "docs"
New-Item -ItemType Directory -Path (Join-Path $stage "tests") -Force | Out-Null
Get-ChildItem -LiteralPath (Join-Path $project "tests") -File |
    Where-Object { $_.Extension -in ".py", ".cpp", ".h" } |
    Copy-Item -Destination (Join-Path $stage "tests") -Force
Copy-Directory "packaging\release_files" "."
Copy-File "packaging\requirements-runtime.txt" "requirements-runtime.txt"
Copy-File "README.md" "README_PROJECT.md"

# Models actually referenced by the teleop/simulation programs.
Copy-Directory "simulation\generated" "simulation\generated"
Copy-Directory "official\TCP-IP-ROS-6AXis-models\dobot_description\meshes\cr5" "official\TCP-IP-ROS-6AXis-models\dobot_description\meshes\cr5"
Copy-File "official\TCP-IP-ROS-6AXis-models\dobot_description\urdf\cr5_robot.urdf" "official\TCP-IP-ROS-6AXis-models\dobot_description\urdf\cr5_robot.urdf"
Copy-File "official\TCP-IP-ROS-6AXis-models\LICENSE" "official\TCP-IP-ROS-6AXis-models\LICENSE"
Copy-Directory "official\PathOnRoboticsOSS\hardware\robots\so101_6dof_wrist\software\robot_assets" "official\PathOnRoboticsOSS\hardware\robots\so101_6dof_wrist\software\robot_assets"
Copy-Directory "uarm_cr10a_teleop_sim\uarm_cr10a_teleop_sim" "models\source\uarm_cr10a_teleop_sim"
Copy-Directory "uarm_cr10a_teleop_sim\LeRobot-Anything-U-Arm-main\mechanical\Feetech_servo\Config2_STL" "models\source\LeRobot-Anything-U-Arm\mechanical\Feetech_servo\Config2_STL"
Copy-File "uarm_cr10a_teleop_sim\LeRobot-Anything-U-Arm-main\LICENSE.txt" "models\source\LeRobot-Anything-U-Arm\LICENSE.txt"
Copy-File "uarm_cr10a_teleop_sim\LeRobot-Anything-U-Arm-main\README.md" "models\source\LeRobot-Anything-U-Arm\README.md"
Copy-File "uarm_cr10a_teleop_sim\LeRobot-Anything-U-Arm-main\README_CN.md" "models\source\LeRobot-Anything-U-Arm\README_CN.md"

# The curated model source contained two local bytecode caches and an old
# manifest that listed one of them. They are not source assets and must not be
# distributed as if they were portable files.
$curatedModel = Join-Path $stage "models\source\uarm_cr10a_teleop_sim"
Get-ChildItem -LiteralPath $curatedModel -Directory -Filter "__pycache__" -Recurse |
    ForEach-Object { Remove-Item -LiteralPath $_.FullName -Recurse -Force }
foreach ($oldManifest in "PACKAGE_MANIFEST.txt", "SHA256SUMS.txt") {
    $oldPath = Join-Path $curatedModel $oldManifest
    if (Test-Path -LiteralPath $oldPath) { Remove-Item -LiteralPath $oldPath -Force }
}

# Rename the four Chinese 3MF source names without placing non-ASCII source
# text in this Windows PowerShell 5.1 script. Contents are not modified.
$config2Models = Join-Path $stage "models\source\LeRobot-Anything-U-Arm\mechanical\Feetech_servo\Config2_STL"
$modelRenames = [ordered]@{}
$modelRenames.Add(([string][char]0x4E2D + [char]0x95F4 + "4" + [char]0x4E2A + ".gcode.3mf"), "middle_4.gcode.3mf")
$modelRenames.Add(([string][char]0x524D + "4" + [char]0x4E2A + ".gcode.3mf"), "front_4.gcode.3mf")
$modelRenames.Add(([string][char]0x7ACB + [char]0x67F1 + ".gcode.3mf"), "pillar.gcode.3mf")
$modelRenames.Add(([string][char]0x7ACB + [char]0x67F1 + [char]0x548C + [char]0x6273 + [char]0x673A + ".gcode.3mf"), "pillar_and_trigger.gcode.3mf")
foreach ($entry in $modelRenames.GetEnumerator()) {
    $sourceModel = Join-Path $config2Models $entry.Key
    if (-not (Test-Path -LiteralPath $sourceModel -PathType Leaf)) {
        throw "Expected Config2 model source is missing."
    }
    Move-Item -LiteralPath $sourceModel -Destination (Join-Path $config2Models $entry.Value)
}

# Firmware source without PlatformIO caches, plus the exact verified binary.
foreach ($name in "tcan485_dm4310_read_probe", "tcan485_dm4310_gateway") {
    foreach ($file in "platformio.ini", "README.md") {
        Copy-File "firmware\$name\$file" "firmware\$name\$file"
    }
    Copy-Directory "firmware\$name\src" "firmware\$name\src"
}
$prebuiltBase = "firmware\tcan485_dm4310_read_probe\.pio\build\esp32dev"
foreach ($file in "firmware.bin", "bootloader.bin", "partitions.bin") {
    Copy-File "$prebuiltBase\$file" "firmware\tcan485_dm4310_read_probe\prebuilt\$file"
}

# Offline Python payload prepared under packaging/cache.
Copy-Directory "packaging\cache\wheels" "wheels"
foreach ($obsoleteWheel in "python_dateutil-4.0.0-py3-none-any.whl", "tzdata-2026.6-py2.py3-none-any.whl") {
    $obsoletePath = Join-Path $stage "wheels\$obsoleteWheel"
    if (Test-Path -LiteralPath $obsoletePath) { Remove-Item -LiteralPath $obsoletePath -Force }
}
Copy-File "packaging\cache\python-3.13.9-amd64.exe" "runtime\python-3.13.9-amd64.exe"

$pythonInstaller = Join-Path $stage "runtime\python-3.13.9-amd64.exe"
$pythonInstallerHash = (Get-FileHash -LiteralPath $pythonInstaller -Algorithm SHA256).Hash
$expectedPythonInstallerHash = "200DDFF856BBFF949D2CC1BE42E8807C07538ABD6B6966D5113A094CF628C5C5"
if ($pythonInstallerHash -ne $expectedPythonInstallerHash) {
    throw "Python 官方安装器哈希不匹配：$pythonInstallerHash"
}
$pythonSignature = Get-AuthenticodeSignature -LiteralPath $pythonInstaller
if ($pythonSignature.Status -ne "Valid" -or
    $pythonSignature.SignerCertificate.Subject -notlike "*Python Software Foundation*") {
    throw "Python 安装器数字签名无效：$($pythonSignature.Status)"
}

$firmware = Join-Path $stage "firmware\tcan485_dm4310_read_probe\prebuilt\firmware.bin"
$firmwareHash = (Get-FileHash -LiteralPath $firmware -Algorithm SHA256).Hash
$expectedFirmwareHash = "BEEF14C39410AA1D4C057CC33DA6659185CBF9101F7EA2515F26FAA5BE2083BF"
if ($firmwareHash -ne $expectedFirmwareHash) {
    throw "预编译固件哈希不匹配：$firmwareHash"
}

$releaseInfo = [ordered]@{
    package = $releaseName
    version = $Version
    release_channel = "prerelease"
    built_at = (Get-Date).ToString("o")
    platform = "Windows x64"
    python = "3.13.9"
    python_installer_sha256 = $pythonInstallerHash
    robot = "Dobot CR5AF / CC262"
    leader = "U-Arm / FE-URT-2"
    gripper_motor = "DM-J4310-2EC V1.2"
    firmware_sha256 = $firmwareHash
    historical_logs_included = $false
    credentials_included = $false
}
$releaseInfo | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $stage "RELEASE_INFO.json") -Encoding UTF8

& python (Join-Path $stage "scripts\verify_release_encoding.py") $stage
if ($LASTEXITCODE -ne 0) { throw "Release encoding validation failed." }

# Refuse to publish the two known classes of machine-specific identifiers that
# were present in early private snapshots. The controller's standard .1 host is
# intentionally allowed because it is a deployment default, not a PC identity.
$textExtensions = ".py", ".md", ".txt", ".json", ".ini", ".ps1", ".bat", ".cmd"
$privacyHits = Get-ChildItem -LiteralPath $stage -File -Recurse |
    Where-Object { $_.Extension.ToLowerInvariant() -in $textExtensions } |
    Select-String -Pattern "5C8428[0-9A-Fa-f]{4}", "192\.168\.201\.10[01]"
if ($privacyHits) {
    $locations = ($privacyHits | ForEach-Object { "$($_.Path):$($_.LineNumber)" }) -join ", "
    throw "Release privacy validation failed: $locations"
}

$hashFile = Join-Path $stage "SHA256SUMS.txt"
$hashLines = Get-ChildItem -LiteralPath $stage -File -Recurse |
    Where-Object { $_.FullName -ne $hashFile } |
    Sort-Object FullName |
    ForEach-Object {
        $relative = $_.FullName.Substring($stage.Length).TrimStart([char[]]"\/").Replace("\", "/")
        "{0} *{1}" -f (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant(), $relative
    }
$hashLines | Set-Content -LiteralPath $hashFile -Encoding UTF8

if (-not $SkipZip) {
    $zip = Join-Path $dist "$releaseName.zip"
    if (Test-Path -LiteralPath $zip) { Remove-Item -LiteralPath $zip -Force }
    Compress-Archive -LiteralPath $stage -DestinationPath $zip -CompressionLevel Optimal
    & python (Join-Path $stage "scripts\verify_release_encoding.py") $stage --zip $zip
    if ($LASTEXITCODE -ne 0) { throw "ZIP encoding validation failed." }
    $zipHash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
    Set-Content -LiteralPath "$zip.sha256" -Value "$zipHash *$([IO.Path]::GetFileName($zip))" -Encoding ASCII
    Write-Host "ZIP: $zip"
    Write-Host "SHA256: $zipHash"
}
Write-Host "FOLDER: $stage"
