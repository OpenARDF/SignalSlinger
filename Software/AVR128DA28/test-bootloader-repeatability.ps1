[CmdletBinding()]
param(
    [string]$Port = 'COM6',

    [int]$Count = 3,

    [int]$BootBaud = 115200,

    [int]$AppBaud = 9600,

    [string]$HexPath = '',

    [string]$OutputDir = '',

    [switch]$SkipBuild,

    [switch]$SkipSerialVerify,

    [switch]$NoVerify,

    [int]$InterRunDelayMs = 1000,

    [string]$Tool = 'atmelice',

    [string]$ToolSerial = 'J41800053674',

    [string]$Interface = 'UPDI',

    [string]$Device = 'avr128da28',

    [string]$Clock = '100kHz'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if($Count -lt 1)
{
    throw '-Count must be at least 1.'
}
if($InterRunDelayMs -lt 0)
{
    throw '-InterRunDelayMs cannot be negative.'
}

$updateScript = Join-Path $PSScriptRoot 'update-firmware-serial.ps1'
if(-not (Test-Path -LiteralPath $updateScript))
{
    throw "Updater script not found: $updateScript"
}

$timestamp = [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')
if([string]::IsNullOrWhiteSpace($OutputDir))
{
    $OutputDir = Join-Path $PSScriptRoot (Join-Path 'tmp\bootloader-repeatability' $timestamp)
}
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

if(-not $SkipBuild)
{
    & (Join-Path $PSScriptRoot 'build-relocated-firmware.ps1') -Configuration Release
    if($LASTEXITCODE -ne 0)
    {
        throw "Relocated firmware build failed with exit code $LASTEXITCODE."
    }
}

$resolvedHexPath = if([string]::IsNullOrWhiteSpace($HexPath)) {
    Join-Path $PSScriptRoot 'SignalSlinger\Release\SignalSlinger.hex'
} else {
    $HexPath
}
if(-not (Test-Path -LiteralPath $resolvedHexPath))
{
    throw "Relocated firmware HEX not found: $resolvedHexPath"
}

$hexHash = (Get-FileHash -LiteralPath $resolvedHexPath -Algorithm SHA256).Hash
$gitCommit = ''
try
{
    $gitCommit = (& git -C $PSScriptRoot rev-parse HEAD 2>$null).Trim()
}
catch
{
    $gitCommit = 'unknown'
}

$results = @()
Write-Host ("Bootloader repeatability test: {0} update(s), port {1}, app {2} baud, bootloader {3} baud, serial verify {4}, UPDI verify {5}" -f $Count, $Port, $AppBaud, $BootBaud, (-not $SkipSerialVerify), (-not $NoVerify))
Write-Host "Output directory: $OutputDir"
Write-Host "HEX: $resolvedHexPath"
Write-Host "HEX SHA256: $hexHash"
Write-Host "Git commit: $gitCommit"

for($run = 1; $run -le $Count; $run++)
{
    Write-Host ("=== Run {0}/{1} ===" -f $run, $Count)
    $runStart = [DateTime]::UtcNow
    $runLogPath = Join-Path $OutputDir ("run-{0:D3}.log" -f $run)
    $arguments = @(
        '-ExecutionPolicy', 'Bypass',
        '-File', $updateScript,
        '-Port', $Port,
        '-BootBaud', $BootBaud,
        '-AppBaud', $AppBaud,
        '-SkipBuild',
        '-RequestBootloaderFromApp',
        '-Tool', $Tool,
        '-ToolSerial', $ToolSerial,
        '-Interface', $Interface,
        '-Device', $Device,
        '-Clock', $Clock
    )

    $arguments += @('-HexPath', $resolvedHexPath)
    if($SkipSerialVerify)
    {
        $arguments += '-SkipSerialVerify'
    }
    if(-not $NoVerify)
    {
        $arguments += '-VerifyWithUpdi'
    }

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $output = & powershell @arguments 2>&1
    $exitCode = $LASTEXITCODE
    $timer.Stop()
    $runFinish = [DateTime]::UtcNow

    $outputLines = @($output | ForEach-Object { [string]$_ })
    $outputLines | Set-Content -LiteralPath $runLogPath -Encoding UTF8
    $outputLines | ForEach-Object { Write-Host $_ }

    $programmingSeconds = $null
    $outputText = ($outputLines | Out-String)
    $match = [regex]::Match($outputText, 'Serial update complete: wrote \d+ pages .* in ([0-9.]+) s')
    if($match.Success)
    {
        $programmingSeconds = [double]::Parse($match.Groups[1].Value, [System.Globalization.CultureInfo]::InvariantCulture)
    }

    $results += [pscustomobject]@{
        Run = $run
        Result = if($exitCode -eq 0) { 'PASS' } else { 'FAIL' }
        ExitCode = $exitCode
        ProgrammingSeconds = $programmingSeconds
        WallSeconds = [Math]::Round($timer.Elapsed.TotalSeconds, 1)
        SerialVerify = (-not $SkipSerialVerify)
        UpdiVerify = (-not $NoVerify)
        StartedUtc = $runStart.ToString('o')
        FinishedUtc = $runFinish.ToString('o')
        LogPath = $runLogPath
    }

    if($exitCode -ne 0)
    {
        throw ("Run {0}/{1} failed with exit code {2}. Log: {3}" -f $run, $Count, $exitCode, $runLogPath)
    }

    if($run -lt $Count -and $InterRunDelayMs -gt 0)
    {
        Start-Sleep -Milliseconds $InterRunDelayMs
    }
}

$programmingTimes = @($results | Where-Object { $null -ne $_.ProgrammingSeconds } | ForEach-Object { $_.ProgrammingSeconds })
$summaryCsvPath = Join-Path $OutputDir 'summary.csv'
$summaryJsonPath = Join-Path $OutputDir 'summary.json'
$metadata = [pscustomobject]@{
    GeneratedUtc = [DateTime]::UtcNow.ToString('o')
    GitCommit = $gitCommit
    HexPath = $resolvedHexPath
    HexSha256 = $hexHash
    Port = $Port
    AppBaud = $AppBaud
    BootBaud = $BootBaud
    Count = $Count
    SerialVerify = (-not $SkipSerialVerify)
    UpdiVerify = (-not $NoVerify)
    Results = $results
}

$results | Export-Csv -LiteralPath $summaryCsvPath -NoTypeInformation
$metadata | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $summaryJsonPath -Encoding UTF8

Write-Host '=== Summary ==='
$results | Format-Table -AutoSize | Out-String | Write-Host
Write-Host "Summary CSV: $summaryCsvPath"
Write-Host "Summary JSON: $summaryJsonPath"

if($programmingTimes.Count -gt 0)
{
    $min = ($programmingTimes | Measure-Object -Minimum).Minimum
    $max = ($programmingTimes | Measure-Object -Maximum).Maximum
    $avg = ($programmingTimes | Measure-Object -Average).Average
    Write-Host ("Programming time: min {0:N1} s, avg {1:N1} s, max {2:N1} s" -f $min, $avg, $max)
}

Write-Host 'Bootloader repeatability test passed.'
