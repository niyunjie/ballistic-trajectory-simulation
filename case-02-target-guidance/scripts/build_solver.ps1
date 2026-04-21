$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $scriptDir
$build = Join-Path $root "build"
$generator = if ($env:CMAKE_GENERATOR) { $env:CMAKE_GENERATOR } else { $null }
$arch = if ($env:CMAKE_GENERATOR_PLATFORM) { $env:CMAKE_GENERATOR_PLATFORM } else { $null }

$toolchainCandidates = @()
if ($env:VCPKG_ROOT) {
  $toolchainCandidates += (Join-Path $env:VCPKG_ROOT "scripts/buildsystems/vcpkg.cmake")
}
$toolchainCandidates += "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"
$toolchain = $toolchainCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

New-Item -ItemType Directory -Force -Path $build | Out-Null

Write-Host "[1/2] Configuring CMake..."
$configureArgs = @("-S", $root, "-B", $build, "--fresh")
if ($generator) {
  $configureArgs += @("-G", $generator)
}
if ($arch -and $generator -and $generator.StartsWith("Visual Studio")) {
  $configureArgs += @("-A", $arch)
}
if ($toolchain) {
  Write-Host "Using vcpkg toolchain:" $toolchain
  $configureArgs += "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
} else {
  Write-Warning "No vcpkg toolchain detected. Set VCPKG_ROOT if osgEarth dependencies are not discoverable."
}
& cmake @configureArgs
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

Write-Host "[2/2] Building ballistic_solver_cli..."
& cmake --build $build --config Release --target ballistic_solver_cli

if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}

Write-Host "Build complete:" (Join-Path $build "Release\ballistic_solver_cli.exe")
