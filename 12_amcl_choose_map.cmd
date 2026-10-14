@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0launchers\localize.ps1" -ChooseMap
if errorlevel 1 pause
