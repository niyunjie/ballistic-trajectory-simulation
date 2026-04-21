@echo off
setlocal

set SCRIPT_DIR=%~dp0

for %%I in ("%SCRIPT_DIR%..") do set ROOT=%%~fI
set BUILD=%ROOT%\build
set APP_DIR=%ROOT%\apps\osgearth_viewer
set EARTH=%APP_DIR%\assets\readymap.earth

if defined CMAKE_GENERATOR (
  set GENERATOR=%CMAKE_GENERATOR%
) else (
  set GENERATOR=
)

if defined CMAKE_GENERATOR_PLATFORM (
  set ARCH=%CMAKE_GENERATOR_PLATFORM%
) else (
  set ARCH=
)

if defined VCPKG_ROOT (
  set TOOLCHAIN=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
) else (
  set TOOLCHAIN=C:\dev\vcpkg\scripts\buildsystems\vcpkg.cmake
)

if not exist "%BUILD%" mkdir "%BUILD%"

set CONFIGURE_ARGS=-S "%ROOT%" -B "%BUILD%" --fresh
if defined GENERATOR set CONFIGURE_ARGS=%CONFIGURE_ARGS% -G "%GENERATOR%"
if defined ARCH if defined GENERATOR (
  echo %GENERATOR% | findstr /B /C:"Visual Studio" >nul && set CONFIGURE_ARGS=%CONFIGURE_ARGS% -A %ARCH%
)
if exist "%TOOLCHAIN%" (
  echo Using vcpkg toolchain: %TOOLCHAIN%
  set CONFIGURE_ARGS=%CONFIGURE_ARGS% -DCMAKE_TOOLCHAIN_FILE=%TOOLCHAIN%
) else (
  echo Warning: no vcpkg toolchain detected. Set VCPKG_ROOT if dependencies are not discoverable.
)

cmake %CONFIGURE_ARGS%
if errorlevel 1 exit /b 1

cmake --build "%BUILD%" --config Release --target ballistic_osgearth
if errorlevel 1 exit /b 1

cd /d "%APP_DIR%"
"%BUILD%\Release\ballistic_osgearth.exe" "%EARTH%"
