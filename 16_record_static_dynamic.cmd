@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\benchmark-localization.ps1"
if errorlevel 1 pause
