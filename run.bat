@echo off
set PATH=C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
cd /d "%~dp0"
echo Starting Robotics Simulator...
"%~dp0build\RoboticsSimulator.exe" %*
if errorlevel 1 (
  echo.
  echo The simulator exited with an error.
  pause
)
