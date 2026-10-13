param([ValidateSet('menu','beautymap','dufomap','conceptgraphs','hovsg')][string]$Method='menu', [int]$Frames=8, [switch]$NoGui, [switch]$NonInteractive)
$ErrorActionPreference='Stop'
if ($Method -eq 'menu') {
  Write-Host 'Fresh native execution: 1 BeautyMap, 2 DUFOMap, 3 ConceptGraphs, 4 HOV-SG'
  $methods=@('beautymap','dufomap','conceptgraphs','hovsg')
  $choice=Read-Host 'Select 1..4'
  if ($choice -notmatch '^[1-4]$') { throw 'Select one method.' }
  $Method=$methods[[int]$choice-1]
}
$repoRoot=Split-Path $PSScriptRoot -Parent
$cachePath=Join-Path $repoRoot '.cache'
[void][IO.Directory]::CreateDirectory($cachePath)
$runLock=[IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
$linuxRoot=(& wsl.exe -d Ubuntu-22.04 -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxRoot.StartsWith('/')) { $runLock.Dispose(); throw 'Cannot resolve WSL path.' }
$name='reproduce-'+$Method+'-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
$runPath=Join-Path $repoRoot ('runs\'+$name)
$linuxRun=$linuxRoot+'/runs/'+$name
[void][IO.Directory]::CreateDirectory((Join-Path $repoRoot 'runs'))
@{path=$runPath;name=$name;pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
try {
  if (-not $NoGui) {
    # Viewer starts once the run exists; algorithms always create fresh output.
    $viewScript="while [ ! -d '$linuxRun' ]; do sleep 0.2; done; exec bash '$linuxRoot/scripts/view.sh' --run '$linuxRun'"
    $arguments=@('-d','Ubuntu-22.04','--','bash','-lc',('"'+$viewScript+'"'))
    Start-Process -FilePath 'wsl.exe' -ArgumentList $arguments -WindowStyle Hidden | Out-Null
  }
  & wsl.exe -d Ubuntu-22.04 -- bash ($linuxRoot+'/scripts/reproduce.sh') --method $Method --output $linuxRun --frames "$Frames"
  if ($LASTEXITCODE -ne 0) { throw 'Native reproduction failed. Inspect this new run directory.' }
  Write-Host ('Complete: '+$runPath) -ForegroundColor Green
  if (-not $NonInteractive) { Read-Host 'Press Enter to close this session' | Out-Null }
} finally {
  if (Test-Path -LiteralPath $runPath) { New-Item -ItemType File -Path (Join-Path $runPath 'STOP') -Force | Out-Null }
  $runLock.Dispose()
}
