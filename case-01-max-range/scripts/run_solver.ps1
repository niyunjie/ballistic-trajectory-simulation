$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $scriptDir
$exe = Join-Path $root "build\Release\ballistic_solver_cli.exe"

if (-not (Test-Path $exe)) {
  Write-Host "Solver executable not found. Starting build..."
  & (Join-Path $scriptDir "build_solver.ps1")
}

Write-Host "Launching solver..."
Write-Host ""
Push-Location $root
& $exe
Pop-Location
