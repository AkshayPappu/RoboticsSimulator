@echo off
REM Main app launcher — Chrono::VSG on the NVIDIA discrete GPU.
REM Isolated Chrono DLLs must come first so the broken MSYS2 Chrono_vsg is not loaded.
REM VSG_FILE_PATH is the Chrono data root (parent of vsg/), matching shader lookups
REM like "vsg/shaders/chronoLineShader.vert".
set PATH=C:\Users\aksha\git\chrono-vsg-install\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
set VSG_FILE_PATH=C:\Users\aksha\git\chrono-vsg-install\data
REM Avoid OBS/Epic/NVIDIA overlay Vulkan layers injecting into the process.
set VK_LOADER_LAYERS_DISABLE=~implicit~
cd /d "%~dp0"
echo Starting Robotics Simulator (Chrono::VSG)...
"%~dp0build\RoboticsSimulator.exe" %*
if errorlevel 1 (
  echo.
  echo The simulator exited with an error.
  pause
)
