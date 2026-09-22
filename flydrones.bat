@echo off
REM ============================================================
REM FlyDrones Launcher Helper
REM Sets PYTHONPATH to include src/ before running commands
REM ============================================================

setlocal
cd /d "%~dp0"
set "PYTHONPATH=%~dp0src;%PYTHONPATH%"

echo [FlyDrones] PYTHONPATH set to: %PYTHONPATH:~0,100%...
echo.

if "%~1"=="" (
    echo Usage:
    echo   %~nx0 demo --live
    echo   %~nx0 demo --record my_demo.gif
    echo   %~nx0 inspect
    echo   %~nx0 swarm --live
    echo   %~nx0 bench
    echo   %~nx0 download malecns
    echo   %~nx0 build-brain --out data/malecns_brain.npz
    echo   %~nx0 fly --drone tello ...
    echo.
    echo   Or pass any flydrones command and arguments.
    exit /b 0
)

python -m flydrones %*
endlocal
