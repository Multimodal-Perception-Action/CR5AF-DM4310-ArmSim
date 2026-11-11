@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launch.ps1" -Mode teleop
set "LAUNCH_RESULT=%ERRORLEVEL%"
if not "%LAUNCH_RESULT%"=="0" pause
exit /b %LAUNCH_RESULT%
