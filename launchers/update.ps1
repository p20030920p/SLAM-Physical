param([switch]$NonInteractive)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
Push-Location $repoRoot
try {
  $status=& git status --porcelain
  if ($LASTEXITCODE -ne 0) { throw 'This folder must be a Git checkout.' }
  if ($status) { throw 'Save or commit local changes before updating. No files were overwritten.' }
  $branch=(& git branch --show-current | Out-String).Trim()
  if ($branch -ne 'main') { throw 'Switch to main before updating.' }
  & git -c http.proxy= pull --ff-only origin main
  if ($LASTEXITCODE -ne 0) { throw 'Update failed; local changes were preserved.' }
  Write-Host 'Updated. Existing runs and algorithm environments are preserved.' -ForegroundColor Green
  if (-not $NonInteractive) { Read-Host 'Press Enter' | Out-Null }
} finally { Pop-Location }
