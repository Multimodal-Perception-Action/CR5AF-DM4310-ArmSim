@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0verify_package.ps1"
set "CHECK_RESULT=%ERRORLEVEL%"
pause
exit /b %CHECK_RESULT%
