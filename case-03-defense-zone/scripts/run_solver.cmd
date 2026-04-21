@echo off
setlocal

set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%..") do set ROOT=%%~fI
set EXE=%ROOT%\build\Release\ballistic_solver_cli.exe

if not exist "%EXE%" (
  echo Solver executable not found. Starting build...
  call "%SCRIPT_DIR%build_solver.cmd"
  if errorlevel 1 exit /b 1
)

echo Launching solver...
echo.
cd /d "%ROOT%"
"%EXE%"
