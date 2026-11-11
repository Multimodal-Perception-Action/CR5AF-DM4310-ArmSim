@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_tests.ps1"
set "TEST_RESULT=%ERRORLEVEL%"
pause
exit /b %TEST_RESULT%
