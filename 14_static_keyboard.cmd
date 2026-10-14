@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\scene-keyboard.ps1" -Scenario static
if errorlevel 1 pause
