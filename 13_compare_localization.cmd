@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\compare-localization.ps1"
if errorlevel 1 pause
