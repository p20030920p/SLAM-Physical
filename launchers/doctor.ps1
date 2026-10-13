param([switch]$NonInteractive)
$ErrorActionPreference='Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$linuxRoot = (& wsl.exe -d Ubuntu-22.04 -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
& wsl.exe -d Ubuntu-24.04 -u root -- bash ($linuxRoot+'/scripts/doctor_simulation.sh')
if ($LASTEXITCODE -ne 0) { throw 'Run 00_setup.cmd first.' }
& wsl.exe -d Ubuntu-22.04 -- bash ($linuxRoot+'/scripts/doctor.sh')
if ($LASTEXITCODE -ne 0) { throw 'Algorithm or RViz environment check failed.' }
Write-Host 'All required environments are ready.' -ForegroundColor Green
if (-not $NonInteractive) { Read-Host 'Press Enter' | Out-Null }
