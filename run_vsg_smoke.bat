@echo off
set PATH=C:\msys64\ucrt64\bin;C:\msys64\usr\bin;%PATH%
set VSG_FILE_PATH=C:\msys64\ucrt64\share\chrono\data\vsg
set VK_DRIVER_FILES=C:\Windows\System32\DriverStore\FileRepository\iigd_dch_d.inf_amd64_d7a6a2017c0120e7\igvk64.json
set VK_ICD_FILENAMES=%VK_DRIVER_FILES%
cd /d "%~dp0"
echo Forcing Vulkan onto Intel Arc A370M via discrete ICD
"%~dp0build\vsg_smoke.exe"
echo exit=%ERRORLEVEL%
pause
