@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\build-maps.ps1"
if errorlevel 1 pause
