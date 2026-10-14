param([string]$Source='', [int]$Frames=40, [int]$SemanticFrames=8, [string[]]$Methods=@('raw','beautymap','dufomap','conceptgraphs','hovsg'), [switch]$NonInteractive)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
if (-not $Source) {
  $recordings=@(Get-ChildItem -LiteralPath (Join-Path $repoRoot 'runs') -Directory -Filter 'amcl-live-*' | Where-Object {Test-Path -LiteralPath (Join-Path $_.FullName 'mapping_scans.npz')} | Sort-Object Name -Descending)
  if (-not $recordings.Count) {throw 'Finish a drive using 10_amcl_keyboard.cmd first.'}
  for($i=0;$i -lt $recordings.Count;$i++) {Write-Host ("{0}: {1}" -f ($i+1),$recordings[$i].Name)}
  $choice=Read-Host 'Choose the MAPPING drive number (a separate drive will be used for testing)'
  if ($choice -notmatch '^\d+$' -or [int]$choice -lt 1 -or [int]$choice -gt $recordings.Count) {throw 'Invalid recording'}
  $Source=$recordings[[int]$choice-1].FullName
}
$cachePath=Join-Path $repoRoot '.cache'
$runLock=[IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
$linuxRoot=(& wsl.exe -d Ubuntu-22.04 -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
$linuxSource=(& wsl.exe -d Ubuntu-22.04 -- wslpath -u $Source.Replace('\','/') | Out-String).Trim()
$name='localization-maps-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
$runPath=Join-Path $repoRoot ('runs\'+$name)
@{path=$runPath;name=$name;pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
try {
  & wsl.exe -d Ubuntu-22.04 -- bash ($linuxRoot+'/scripts/build_localization_maps.sh') --source $linuxSource --output ($linuxRoot+'/runs/'+$name) --frames "$Frames" --semantic-frames "$SemanticFrames" --methods @Methods
  if ($LASTEXITCODE -ne 0) {throw 'Map build failed'}
  $result=Get-Content -LiteralPath (Join-Path $runPath 'maps.json') -Raw | ConvertFrom-Json
  $failed=@($result.methods.PSObject.Properties | Where-Object {$_.Value.status -ne 'completed'})
  Write-Host ('Map build results: '+$runPath) -ForegroundColor Green
  if ($failed.Count) {Write-Warning ('Some native methods failed: '+(($failed | ForEach-Object {$_.Name}) -join ', ')+'. Inspect maps.json and native.log. Failed maps are not offered for localization.')}
  if (-not $NonInteractive) {Read-Host 'Press Enter' | Out-Null}
} finally {$runLock.Dispose()}
