param([string]$Benchmark='')
$ErrorActionPreference='Stop'
$repoRoot=Split-Path $PSScriptRoot -Parent
if (-not $Benchmark) {
  $latest=Get-Content -LiteralPath (Join-Path $repoRoot '.cache\localization-benchmark-latest.json') -Raw | ConvertFrom-Json
  $Benchmark=$latest.path
}
$state=Get-Content -LiteralPath (Join-Path $Benchmark 'benchmark.json') -Raw | ConvertFrom-Json
if ($state.status -ne 'completed') {throw 'Export requires a fully completed benchmark.'}
$destination=Join-Path $repoRoot 'docs\results\static-dynamic'
[void][IO.Directory]::CreateDirectory($destination)
foreach($name in @('REPORT.zh-CN.md','summary.csv','summary.json','map-gallery.png','lidar-map-comparison.png','localization-comparison.png','dynamic-localization-replay.mp4','video.json','benchmark.json')) {
  Copy-Item -LiteralPath (Join-Path $Benchmark $name) -Destination (Join-Path $destination $name) -Force
}
foreach($scene in @('static','dynamic')) {
  $mapsDestination=Join-Path $destination ('maps-'+$scene)
  [void][IO.Directory]::CreateDirectory($mapsDestination)
  foreach($method in @('raw','beautymap','dufomap','rgbd-raw','conceptgraphs','hovsg')) {
    $target=Join-Path $mapsDestination $method
    [void][IO.Directory]::CreateDirectory($target)
    foreach($file in @('map.pgm','map.yaml','projection.json')) {
      Copy-Item -LiteralPath (Join-Path $Benchmark ('maps-'+$scene+'\maps\'+$method+'\'+$file)) -Destination (Join-Path $target $file) -Force
    }
  }
  Copy-Item -LiteralPath (Join-Path $Benchmark ('maps-'+$scene+'\maps.json')) -Destination (Join-Path $destination ('maps-'+$scene+'.json')) -Force
}
foreach($condition in @('static','dynamic','clean-map-dynamic-test')) {
  foreach($extension in @('csv','json')) {
    Copy-Item -LiteralPath (Join-Path $Benchmark ('comparison-'+$condition+'\comparison.'+$extension)) -Destination (Join-Path $destination ('comparison-'+$condition+'.'+$extension)) -Force
  }
}
$reportText=(Get-Content -LiteralPath (Join-Path $Benchmark 'REPORT.zh-CN.md') -Raw -Encoding UTF8).TrimEnd()
$lastParagraph=$reportText.LastIndexOf("`n`n")
if ($lastParagraph -lt 0) {throw 'Unexpected report structure'}
$portableText=$reportText.Substring(0,$lastParagraph)+"`n`nMeasured tables: [summary.csv](summary.csv), [summary.json](summary.json), comparison-static.csv, comparison-dynamic.csv and comparison-clean-map-dynamic-test.csv.`n`n[Recorded-data reconstruction video](dynamic-localization-replay.mp4). [Four-category GIF/MP4 gallery with frozen live traces and timing checks](https://github.com/p20030920p/SLAM_Learning/tree/main/results/localization/static-dynamic). Raw bags, scans, RGB-D, native artifacts and full per-repeat traces remain in the local runs folder. Original benchmark: $(Split-Path $Benchmark -Leaf).`n"
$portableText | Set-Content -LiteralPath (Join-Path $destination 'README.md') -Encoding UTF8
$portableText | Set-Content -LiteralPath (Join-Path $destination 'REPORT.zh-CN.md') -Encoding UTF8
@{benchmark=(Split-Path $Benchmark -Leaf);local_path=$Benchmark;pose_source=$state.pose_source;status='completed';raw_data='local runs folder; not included in portable evidence'} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $destination 'session-index.json') -Encoding UTF8
Write-Host ('Exported measured results: '+$destination)
