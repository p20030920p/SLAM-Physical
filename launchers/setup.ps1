$ErrorActionPreference='Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$distroNames = (& wsl.exe --list --quiet | Out-String).Replace([char]0,'')
if ($distroNames -notmatch 'Ubuntu-24.04') {
  & wsl.exe --install --distribution Ubuntu-24.04 --no-launch --web-download
  if ($LASTEXITCODE -ne 0) { throw 'Ubuntu-24.04 installation failed.' }
}
$linuxRoot = (& wsl.exe -d Ubuntu-22.04 -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
& wsl.exe -d Ubuntu-24.04 -u root -- bash ($linuxRoot+'/scripts/setup_simulation.sh')
if ($LASTEXITCODE -ne 0) { throw 'Simulation dependency installation failed.' }
& wsl.exe -d Ubuntu-24.04 -u root -- bash ($linuxRoot+'/scripts/setup_localization.sh')
if ($LASTEXITCODE -ne 0) { throw 'Localization dependency installation failed.' }
& wsl.exe -d Ubuntu-22.04 -- bash ($linuxRoot+'/scripts/fetch_sim2real.sh')
if ($LASTEXITCODE -ne 0) { throw 'Pinned simulation assets are unavailable.' }
Write-Host 'Simulation setup complete. Existing algorithm environments are reused.'
