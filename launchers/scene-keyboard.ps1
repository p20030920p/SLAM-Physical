param([ValidateSet('static','dynamic')][string]$Scenario='static', [string]$Method='', [switch]$NoGui,
      [switch]$NoKeyboard, [switch]$NonInteractive, [double]$Duration=0)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
$latestPath=Join-Path $repoRoot '.cache\localization-benchmark-latest.json'
if (Test-Path -LiteralPath $latestPath) {
  $latest=Get-Content -LiteralPath $latestPath -Raw | ConvertFrom-Json
  $mapsRoot=Join-Path $latest.path ('maps-'+$Scenario+'\maps')
} else {$mapsRoot=Join-Path $repoRoot ('docs\results\static-dynamic\maps-'+$Scenario)}
$methods=@('raw','beautymap','dufomap','rgbd-raw','conceptgraphs','hovsg')
if (-not $Method) {
  if ($Scenario -eq 'static') {$Method='raw'} else {
    Write-Host 'Dynamic environment: select the map to use with the SAME AMCL.' -ForegroundColor Cyan
    $available=@($methods | Where-Object {Test-Path -LiteralPath (Join-Path $mapsRoot ($_+'\map.yaml'))})
    if (-not $available.Count) {throw 'Run 16_record_static_dynamic.cmd to prepare the maps first.'}
    for($i=0;$i -lt $available.Count;$i++) {Write-Host ("{0}: {1}" -f ($i+1),$available[$i])}
    Write-Host 'raw = no paper algorithm (LiDAR); rgbd-raw = no paper algorithm (RGB-D).'
    $choice=Read-Host 'Choose a map number (Enter = raw)'
    if (-not $choice) {$Method='raw'} elseif ($choice -match '^\d+$' -and [int]$choice -ge 1 -and [int]$choice -le $available.Count) {$Method=$available[[int]$choice-1]} else {throw 'Invalid map choice'}
  }
}
if ($Method -notin $methods) {throw 'Unknown mapping method'}
$map=Join-Path $mapsRoot ($Method+'\map.yaml')
if (-not (Test-Path -LiteralPath $map)) {throw ('Map missing: '+$map+'. Run 16_record_static_dynamic.cmd first.')}
Write-Host ('Scene: '+$Scenario+'; map: '+$Method) -ForegroundColor Green
$options=@{Scenario=$Scenario;MapYaml=$map;Duration=$Duration;ActorPhase=5.0}
if ($NoGui) {$options.NoGui=$true};if ($NoKeyboard) {$options.NoKeyboard=$true};if ($NonInteractive) {$options.NonInteractive=$true}
& (Join-Path $PSScriptRoot 'localize.ps1') @options
