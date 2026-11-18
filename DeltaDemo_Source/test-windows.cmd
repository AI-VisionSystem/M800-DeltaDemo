@echo off
setlocal
if not exist "%~dp0build-temp" mkdir "%~dp0build-temp"
set "TEMP=%~dp0build-temp"
set "TMP=%TEMP%"
set QT_FORCE_STDERR_LOGGING=1
rem Use the same x64 MSVC + Qt environment as build-windows.cmd.
where nmake >nul 2>nul
if errorlevel 1 (echo MSVC nmake not found. & exit /b 1)
where qmake >nul 2>nul
if errorlevel 1 (echo Qt qmake not found. & exit /b 1)
for /f "delims=" %%Q in ('qmake -query QT_INSTALL_BINS') do set "DEMO_QT_BIN=%%Q"
pushd "%~dp0"
if not exist build-tests mkdir build-tests
if not exist build-tests\settings mkdir build-tests\settings
if not exist build-tests\controller mkdir build-tests\controller
pushd build-tests\settings
qmake ..\..\tests\settings_test.pro CONFIG+=release CONFIG-=debug
if errorlevel 1 goto failed
nmake
if errorlevel 1 goto failed
copy /y "%DEMO_QT_BIN%\Qt5Core.dll" . >nul
if errorlevel 1 goto failed
settings_test.exe
if not "%errorlevel%"=="0" goto failed
popd
pushd build-tests\controller
qmake ..\..\tests\controller_test.pro CONFIG+=release CONFIG-=debug
if errorlevel 1 goto failed
nmake
if errorlevel 1 goto failed
copy /y "%DEMO_QT_BIN%\Qt5Core.dll" . >nul
if errorlevel 1 goto failed
copy /y "%DEMO_QT_BIN%\Qt5Network.dll" . >nul
if errorlevel 1 goto failed
controller_test.exe
if not "%errorlevel%"=="0" goto failed
popd
popd
echo All offline tests passed. No controller was accessed.
exit /b 0
:failed
echo Offline test build or execution failed.
exit /b 1
