param([switch]$NoGui, [switch]$NonInteractive, [int]$Frames=40, [int]$RevisionAfter=16,
      [ValidateSet('static','departure')][string]$Scenario='static')
$ErrorActionPreference='Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$cachePath = Join-Path $repoRoot '.cache'
[void][IO.Directory]::CreateDirectory($cachePath)
$runLock = [IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
$runName = $Scenario + '-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,6)
$runPath = Join-Path $repoRoot ('runs\' + $runName)
[void][IO.Directory]::CreateDirectory($runPath)
$linuxRoot = (& wsl.exe -d Ubuntu-22.04 -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxRoot.StartsWith('/')) { $runLock.Dispose(); throw 'Cannot resolve the repository path in WSL.' }
$linuxRun = $linuxRoot + '/runs/' + $runName
@{path=$runPath;name=$runName;pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
try {
  Write-Host 'Stage 1/3: actual Gazebo acquisition' -ForegroundColor Cyan
  if (-not $NoGui) {
    $viewerArgs = @('-d','Ubuntu-22.04','--','bash',('"'+$linuxRoot+'/scripts/view.sh"'),'--run',('"'+$linuxRun+'"'))
    Start-Process -FilePath 'wsl.exe' -ArgumentList $viewerArgs -WindowStyle Hidden | Out-Null
  }
  $captureArgs = @('-d','Ubuntu-24.04','-u','root','--','bash',($linuxRoot+'/scripts/capture.sh'),'--output',($linuxRun+'/capture'),'--frames',"$Frames",'--scenario',$Scenario)
  if (-not $NoGui) { $captureArgs += '--gui' }
  & wsl.exe @captureArgs
  if ($LASTEXITCODE -ne 0) { throw 'Gazebo acquisition failed. See the run logs.' }
  Write-Host 'Stage 2/3: native BeautyMap and C1/C2/C4 comparison' -ForegroundColor Cyan
  & wsl.exe -d Ubuntu-22.04 -- bash ($linuxRoot+'/scripts/experiment.sh') --dataset ($linuxRun+'/capture') --output ($linuxRun+'/comparison') --revision-after "$RevisionAfter"
  if ($LASTEXITCODE -ne 0) { throw 'BeautyMap experiment failed. See native.log.' }
  Write-Host ('Stage 3/3: complete. Results: '+$runPath) -ForegroundColor Green
  if (-not $NonInteractive) { Read-Host 'Press Enter to close this session and its RViz window' | Out-Null }
} finally {
  New-Item -ItemType File -Path (Join-Path $runPath 'STOP') -Force | Out-Null
  $runLock.Dispose()
}
