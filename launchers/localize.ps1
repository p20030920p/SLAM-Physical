param([string]$MapYaml='', [switch]$ChooseMap, [switch]$NoGui, [switch]$NoKeyboard,
      [switch]$NonInteractive, [double]$Duration=0, [switch]$Autodrive,
      [double]$OdomLinearScale=1.0, [double]$OdomAngularScale=1.0,
      [ValidateSet('static','dynamic')][string]$Scenario='static',
      [double]$ActorPhase=0, [ValidateSet('legacy','mapping','test')][string]$Route='legacy')
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
$cachePath=Join-Path $repoRoot '.cache'
[void][IO.Directory]::CreateDirectory($cachePath)
if ($ChooseMap) {
  $maps=@(Get-ChildItem -LiteralPath (Join-Path $repoRoot 'runs') -Recurse -Filter map.yaml | Where-Object { $_.FullName -match '[\\/]maps[\\/]' } | Sort-Object LastWriteTime -Descending)
  if (-not $maps.Count) {throw 'Run 11_build_localization_maps.cmd after your first mapping drive.'}
  for($i=0;$i -lt $maps.Count;$i++) {Write-Host ("{0}: {1}" -f ($i+1),$maps[$i].FullName)}
  $choice=Read-Host 'Choose a map number'
  if ($choice -notmatch '^\d+$' -or [int]$choice -lt 1 -or [int]$choice -gt $maps.Count) {throw 'Invalid map selection'}
  $MapYaml=$maps[[int]$choice-1].FullName
}
$runLock=[IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
$linuxRoot=(& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $repoRoot.Replace('\','/') | Out-String).Trim()
$name='amcl-live-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
$runPath=Join-Path $repoRoot ('runs\'+$name)
$linuxRun=$linuxRoot+'/runs/'+$name
@{path=$runPath;name=$name;pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
$stdout=Join-Path $cachePath ($name+'.log')
$stderr=Join-Path $cachePath ($name+'.stderr.log')
$backend=$null
try {
  $arguments=@('-d','Ubuntu-24.04','-u','root','--','bash',('"'+$linuxRoot+'/scripts/localization.sh"'),'--output',('"'+$linuxRun+'"'),
               '--duration',"$Duration",'--odom-linear-scale',"$OdomLinearScale",'--odom-angular-scale',"$OdomAngularScale",
               '--scenario',$Scenario,'--actor-phase',"$ActorPhase",'--route',$Route)
  if ($NoGui) {$arguments+='--no-gui'}
  if ($Autodrive) {$arguments+='--autodrive'}
  if ($MapYaml) {
    $linuxMap=(& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $MapYaml.Replace('\','/') | Out-String).Trim()
    $arguments+=@('--map',('"'+$linuxMap+'"'))
  }
  Write-Host 'Starting physics robot, AMCL, live RViz errors and automatic ROS bag recording.' -ForegroundColor Cyan
  Write-Host ('Scene: '+$Scenario)
  if ($Scenario -eq 'dynamic') {Write-Host 'Orange boxes show independently observed moving obstacles.'}
  Write-Host 'Focus the keyboard terminal. Hold i to move, j/l to turn, k to stop. Ctrl+C there ends this recording.'
  if (-not $MapYaml) {Write-Host 'Map: SDF reference baseline. Use 12_amcl_choose_map.cmd for an algorithm map.'}
  $backend=Start-Process -FilePath 'wsl.exe' -ArgumentList $arguments -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
  # Cache the native handle so Windows PowerShell 5.1 retains the exit code.
  [void]$backend.Handle
  $printed=0
  $keyboardStarted=$false
  while (-not $backend.HasExited) {
    if (Test-Path -LiteralPath $stdout) {
      $lines=@(Get-Content -LiteralPath $stdout)
      if ($lines.Count -gt $printed) {$lines[$printed..($lines.Count-1)] | ForEach-Object {Write-Host $_};$printed=$lines.Count}
    }
    if (-not $NoKeyboard -and -not $keyboardStarted -and (Test-Path -LiteralPath (Join-Path $runPath 'ready.json'))) {
      $keyboardArguments=@('-w','new','--title','"AMCL keyboard - hold i / j / l, k stops"','wsl.exe','-d','Ubuntu-24.04','-u','root','--','bash',
                           ('"'+$linuxRoot+'/scripts/keyboard.sh"'),('"'+$linuxRun+'"'))
      # A visible interactive terminal is explicitly requested for keyboard driving.
      Start-Process -FilePath 'wt.exe' -ArgumentList $keyboardArguments -WindowStyle Normal | Out-Null
      $keyboardStarted=$true
    }
    Start-Sleep -Milliseconds 400
    $backend.Refresh()
  }
  $backend.WaitForExit()
  if (Test-Path -LiteralPath $stdout) {Get-Content -LiteralPath $stdout | Select-Object -Skip $printed | ForEach-Object {Write-Host $_}}
  if ($backend.ExitCode -ne 0) {Get-Content -LiteralPath $stderr -Tail 20;throw ('Localization failed. Inspect '+$runPath)}
  Write-Host ('Saved bag, CSV errors, figure and mapping inputs: '+$runPath) -ForegroundColor Green
  if (-not $NonInteractive) {Read-Host 'Press Enter' | Out-Null}
} finally {
  if (Test-Path -LiteralPath $runPath) {New-Item -ItemType File -Path (Join-Path $runPath 'STOP') -Force | Out-Null}
  if ($backend -and -not $backend.HasExited) {$backend.WaitForExit(30000) | Out-Null}
  $runLock.Dispose()
}
