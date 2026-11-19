@echo off
REM VSG smoke test against the isolated from-source Chrono build
REM (C:\Users\aksha\git\chrono-vsg-install). Puts those DLLs first on PATH
REM so the defective MSYS2 Chrono_vsg is never loaded. Does NOT force a
REM Vulkan ICD — the NVIDIA discrete GPU is selected normally.
REM
REM VSG_FILE_PATH must be the Chrono *data* root (parent of the vsg/ folder)
REM because Chrono looks up paths like "vsg/shaders/chronoLineShader.vert".
set PATH=C:\Users\aksha\git\chrono-vsg-install\bin;C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
set VSG_FILE_PATH=C:\Users\aksha\git\chrono-vsg-install\data
cd /d "%~dp0"
if exist "%~dp0vsg_smoke_proj\build\vsg_smoke.exe" (
  set SMOKE_EXE=%~dp0vsg_smoke_proj\build\vsg_smoke.exe
) else (
  set SMOKE_EXE=%~dp0build\vsg_smoke.exe
)
echo Running Chrono::VSG smoke test on the default (NVIDIA) Vulkan device
echo exe=%SMOKE_EXE%
"%SMOKE_EXE%"
echo exit=%ERRORLEVEL%
pause
