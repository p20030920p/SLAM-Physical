param([string]$Resume='', [double]$MappingDuration=40, [double]$TestDuration=26,
      [ValidateRange(1,20)][int]$Repeats=3, [int]$Frames=40, [int]$SemanticFrames=24,
      [ValidateSet('provided','wheel')][string]$PoseSource='provided',
      [switch]$NonInteractive)
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
$cachePath=Join-Path $repoRoot '.cache'
[void][IO.Directory]::CreateDirectory($cachePath)
$runLock=[IO.File]::Open((Join-Path $cachePath 'run.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
if ($Resume) {$runPath=[IO.Path]::GetFullPath($Resume)} else {
  $name='static-dynamic-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
  $runPath=Join-Path $repoRoot ('runs\'+$name)
  [void][IO.Directory]::CreateDirectory($runPath)
}
$allowed=[IO.Path]::GetFullPath((Join-Path $repoRoot 'runs'))+[IO.Path]::DirectorySeparatorChar
if (-not $runPath.StartsWith($allowed,[StringComparison]::OrdinalIgnoreCase)) {throw 'Benchmark must be inside this repository runs folder.'}
$statePath=Join-Path $runPath 'benchmark.json'
if (Test-Path -LiteralPath $statePath) {
  $state=Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
  $MappingDuration=$state.mapping_duration_sim_s; $TestDuration=$state.test_duration_sim_s
  $Repeats=$state.repeats; $Frames=$state.frames; $SemanticFrames=$state.semantic_frames
  if ($state.PSObject.Properties.Name -contains 'pose_source') {$PoseSource=$state.pose_source} else {$state | Add-Member -NotePropertyName pose_source -NotePropertyValue $PoseSource}
  if ($Resume -and (Test-Path -LiteralPath (Join-Path $runPath 'STOP'))) {
    Remove-Item -LiteralPath (Join-Path $runPath 'STOP')
  }
} else {
  $state=[pscustomobject]@{status='starting';mapping_duration_sim_s=$MappingDuration;test_duration_sim_s=$TestDuration;
    repeats=$Repeats;frames=$Frames;semantic_frames=$SemanticFrames;pose_source=$PoseSource;stages=@();
    protocol='Independent mapping A and test B per scene; identical B for all maps; repeated fresh AMCL. Extra clean-map/dynamic-test control.'}
}
function Save-State {$state | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $statePath -Encoding UTF8}
function Linux-Path([string]$Path) {
  return (& wsl.exe -d Ubuntu-24.04 -u root -- wslpath -u $Path.Replace('\','/') | Out-String).Trim()
}
$linuxRoot=Linux-Path $repoRoot
$linuxRun=Linux-Path $runPath
@{path=$runPath;name=(Split-Path $runPath -Leaf);pid=$PID} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'active.json') -Encoding UTF8
function Stage([string]$Name,[string]$Distro,[string]$Script,[string[]]$Arguments) {
  if (@($state.stages | Where-Object {$_.name -eq $Name -and $_.status -eq 'completed'}).Count) {
    Write-Host ('SKIP completed stage: '+$Name);return
  }
  if (Test-Path -LiteralPath (Join-Path $runPath 'STOP')) {throw 'Benchmark stop requested. Completed recordings remain saved.'}
  $outputOption=[array]::IndexOf($Arguments,'--output')
  if ($outputOption -ge 0) {
    $linuxOutput=$Arguments[$outputOption+1]
    if (-not $linuxOutput.StartsWith($linuxRun+'/')) {throw 'Stage output is outside this benchmark.'}
    $stageFolder=Join-Path $runPath $linuxOutput.Substring($linuxRun.Length+1).Replace('/','\')
    if (Test-Path -LiteralPath $stageFolder) {
      $archiveFolder=$stageFolder+'.incomplete-'+(Get-Date -Format 'yyyyMMdd-HHmmss')
      $safePrefix=[IO.Path]::GetFullPath($runPath)+[IO.Path]::DirectorySeparatorChar
      if (-not [IO.Path]::GetFullPath($stageFolder).StartsWith($safePrefix) -or -not [IO.Path]::GetFullPath($archiveFolder).StartsWith($safePrefix)) {throw 'Invalid stage archive path'}
      Move-Item -LiteralPath $stageFolder -Destination $archiveFolder
      Write-Host ('Preserved incomplete attempt: '+$archiveFolder)
    }
  }
  Write-Host ('BENCHMARK STAGE: '+$Name) -ForegroundColor Cyan
  $logPath=Join-Path $runPath ($Name+'.log')
  $options=@('-d',$Distro)
  if ($Distro -eq 'Ubuntu-24.04') {$options+=@('-u','root')}
  $savedPreference=$ErrorActionPreference
  $ErrorActionPreference='Continue'
  try {
    & wsl.exe @options -- bash ($linuxRoot+'/scripts/'+$Script) @Arguments 2>&1 | Tee-Object -FilePath $logPath
    $stageExit=$LASTEXITCODE
  } finally {$ErrorActionPreference=$savedPreference}
  if ($stageExit -ne 0) {throw ('Stage failed: '+$Name+'. Inspect '+$logPath)}
  if ($Script -eq 'build_localization_maps.sh') {
    $mapState=Get-Content -LiteralPath (Join-Path $stageFolder 'maps.json') -Raw | ConvertFrom-Json
    $bad=@($mapState.methods.PSObject.Properties | Where-Object {$_.Value.status -ne 'completed'})
    if (@($mapState.methods.PSObject.Properties).Count -ne 6 -or $bad.Count) {throw ('Incomplete native map set: '+$Name+'. Inspect maps.json and native logs.')}
  }
  $state.stages=@($state.stages)+[pscustomobject]@{name=$Name;status='completed';log=$logPath}
  Save-State
}
try {
  $state.status='running';Save-State
  foreach($scene in @('static','dynamic')) {
    Stage ($scene+'-mapping') 'Ubuntu-24.04' 'localization.sh' @('--output',($linuxRun+'/recordings/'+$scene+'-mapping'),
      '--scenario',$scene,'--route','mapping','--autodrive','--duration',"$MappingDuration",'--no-gui','--actor-phase','0')
    Stage ($scene+'-test') 'Ubuntu-24.04' 'localization.sh' @('--output',($linuxRun+'/recordings/'+$scene+'-test'),
      '--scenario',$scene,'--route','test','--autodrive','--duration',"$TestDuration",'--no-gui','--actor-phase','5')
    if ($PoseSource -eq 'provided') {
      Stage ($scene+'-provided-poses') 'Ubuntu-24.04' 'provide_mapping_poses.sh' @('--source',($linuxRun+'/recordings/'+$scene+'-mapping'))
    }
    Stage ($scene+'-maps') 'Ubuntu-22.04' 'build_localization_maps.sh' @('--source',($linuxRun+'/recordings/'+$scene+'-mapping'),
      '--output',($linuxRun+'/maps-'+$scene),'--frames',"$Frames",'--semantic-frames',"$SemanticFrames",'--pose-source',$PoseSource)
  }
  foreach($scene in @('static','dynamic')) {
    Stage ($scene+'-comparison') 'Ubuntu-24.04' 'compare_localization.sh' @('--maps',($linuxRun+'/maps-'+$scene),
      '--recording',($linuxRun+'/recordings/'+$scene+'-test'),'--output',($linuxRun+'/comparison-'+$scene),'--repeats',"$Repeats")
  }
  Stage 'clean-map-dynamic-test' 'Ubuntu-24.04' 'compare_localization.sh' @('--maps',($linuxRun+'/maps-static'),
    '--recording',($linuxRun+'/recordings/dynamic-test'),'--output',($linuxRun+'/comparison-clean-map-dynamic-test'),'--repeats','1')
  Stage 'report' 'Ubuntu-24.04' 'localization_benchmark_report.sh' @('--benchmark',$linuxRun)
  Stage 'video' 'Ubuntu-22.04' 'localization_benchmark_video.sh' @('--benchmark',$linuxRun)
  $state.status='completed';Save-State
  @{path=$runPath;status='completed'} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $cachePath 'localization-benchmark-latest.json') -Encoding UTF8
  Write-Host ('Completed benchmark with recordings, maps and comparisons: '+$runPath) -ForegroundColor Green
  if (-not $NonInteractive) {Read-Host 'Press Enter' | Out-Null}
} catch {
  $state.status='failed';Save-State;throw
} finally {$runLock.Dispose()}
