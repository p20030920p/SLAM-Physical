@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\reproduce.ps1" %*
if errorlevel 1 pause
