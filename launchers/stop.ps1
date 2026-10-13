$repoRoot = Split-Path $PSScriptRoot -Parent
$activePath = Join-Path $repoRoot '.cache\active.json'
if (Test-Path -LiteralPath $activePath) {
  $active = Get-Content -LiteralPath $activePath -Raw | ConvertFrom-Json
  $runPath = [IO.Path]::GetFullPath($active.path)
  $allowed = [IO.Path]::GetFullPath((Join-Path $repoRoot 'runs')) + [IO.Path]::DirectorySeparatorChar
  if (-not $runPath.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)) { throw 'Invalid session path.' }
  New-Item -ItemType File -Path (Join-Path $runPath 'STOP') -Force | Out-Null
  Write-Host 'Stop requested for this experiment only.'
}
