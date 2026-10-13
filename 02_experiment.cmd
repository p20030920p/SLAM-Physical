@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\experiment.ps1" %*
if errorlevel 1 pause
