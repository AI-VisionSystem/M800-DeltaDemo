@echo off
setlocal
if not exist "%~dp0build-temp" mkdir "%~dp0build-temp"
set "TEMP=%~dp0build-temp"
set "TMP=%TEMP%"
rem Run from an x64 MSVC developer prompt with Qt 5.15 msvc2019_64/bin on PATH.
where qmake >nul 2>nul
if errorlevel 1 (echo Qt qmake not found. & exit /b 1)
where nmake >nul 2>nul
if errorlevel 1 (echo Run this script in an x64 MSVC developer prompt. & exit /b 1)
pushd "%~dp0"
if not exist build-windows mkdir build-windows
pushd build-windows
qmake ..\DeltaDemo.pro CONFIG+=release CONFIG-=debug
if errorlevel 1 goto failed
nmake
if errorlevel 1 goto failed
popd
windeployqt --release --no-translations "%~dp0..\DeltaDemo_Runtime\Windows\DeltaDemo.exe"
if errorlevel 1 goto failed
popd
echo Built: DeltaDemo_Runtime\Windows\DeltaDemo.exe
exit /b 0
:failed
echo Build/deployment failed. Check the preceding output.
exit /b 1
