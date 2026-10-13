@echo off
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\experiment.ps1" -Scenario departure
if errorlevel 1 pause
