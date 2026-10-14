param([string]$Maps='', [string]$Recording='', [switch]$Gui, [switch]$NonInteractive, [ValidateRange(1,100)][int]$Repeats=1)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
function Select-Session([string]$RequiredFile,[string]$Prompt) {
  $items=@(Get-ChildItem -LiteralPath (Join-Path $repoRoot 'runs') -Directory | Where-Object {(Test-Path -LiteralPath (Join-Path $_.FullName $RequiredFile)) -and ($RequiredFile -eq 'maps.json' -or $_.Name -like 'amcl-live-*')} | Sort-Object Name -Descending)
  if (-not $items.Count) {throw ('No session containing '+$RequiredFile)}
  for($i=0;$i -lt $items.Count;$i++) {Write-Host ("{0}: {1}" -f ($i+1),$items[$i].Name)}
  $choice=Read-Host $Prompt
  if ($choice -notmatch '^\d+$' -or [int]$choice -lt 1 -or [int]$choice -gt $items.Count) {throw 'Invalid selection'}
  return $items[[int]$choice-1].FullName
}
if (-not $Maps) {$Maps=Select-Session 'maps.json' 'Choose the built map set'}
if (-not $Recording) {$Recording=Select-Session 'bag\metadata.yaml' 'Choose a DIFFERENT completed TEST drive'}
$cachePath=Join-Path $repoRoot '.cache'
$runLock=[IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
$linuxRoot=(& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
$linuxMaps=(& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $Maps.Replace('\','/') | Out-String).Trim()
$linuxRecording=(& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $Recording.Replace('\','/') | Out-String).Trim()
$name='localization-compare-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
$runPath=Join-Path $repoRoot ('runs\'+$name)
@{path=$runPath;name=$name;pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
try {
  $arguments=@('--maps',$linuxMaps,'--recording',$linuxRecording,'--output',($linuxRoot+'/runs/'+$name),'--repeats',"$Repeats")
  if ($Gui) {$arguments+='--gui'}
  & wsl.exe -d Ubuntu-24.04 -u root -- bash ($linuxRoot+'/scripts/compare_localization.sh') @arguments
  if ($LASTEXITCODE -ne 0) {throw ('Replay comparison failed: '+$runPath)}
  Write-Host ('Comparison CSV and per-map reports: '+$runPath) -ForegroundColor Green
  if (-not $NonInteractive) {Read-Host 'Press Enter' | Out-Null}
} finally {$runLock.Dispose()}
