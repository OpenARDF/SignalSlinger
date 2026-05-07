[CmdletBinding()]
param(
    [string]$Port = 'COM6',

    [int]$Count = 3,

    [int]$BootBaud = 115200,

    [int]$AppBaud = 9600,

    [string]$HexPath = '',

    [switch]$SkipBuild,

    [switch]$NoVerify,

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

$updateScript = Join-Path $PSScriptRoot 'update-firmware-serial.ps1'
if(-not (Test-Path -LiteralPath $updateScript))
{
    throw "Updater script not found: $updateScript"
}

if(-not $SkipBuild)
{
    & (Join-Path $PSScriptRoot 'build-relocated-firmware.ps1') -Configuration Release
    if($LASTEXITCODE -ne 0)
    {
        throw "Relocated firmware build failed with exit code $LASTEXITCODE."
    }
}

$results = @()
Write-Host ("Bootloader repeatability test: {0} update(s), port {1}, app {2} baud, bootloader {3} baud, verify {4}" -f $Count, $Port, $AppBaud, $BootBaud, (-not $NoVerify))

for($run = 1; $run -le $Count; $run++)
{
    Write-Host ("=== Run {0}/{1} ===" -f $run, $Count)
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

    if(-not [string]::IsNullOrWhiteSpace($HexPath))
    {
        $arguments += @('-HexPath', $HexPath)
    }
    if(-not $NoVerify)
    {
        $arguments += '-VerifyWithUpdi'
    }

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $output = & powershell @arguments 2>&1
    $exitCode = $LASTEXITCODE
    $timer.Stop()

    $output | ForEach-Object { Write-Host $_ }
    if($exitCode -ne 0)
    {
        throw ("Run {0}/{1} failed with exit code {2}." -f $run, $Count, $exitCode)
    }

    $programmingSeconds = $null
    $outputText = ($output | Out-String)
    $match = [regex]::Match($outputText, 'Serial update complete: wrote \d+ pages .* in ([0-9.]+) s')
    if($match.Success)
    {
        $programmingSeconds = [double]::Parse($match.Groups[1].Value, [System.Globalization.CultureInfo]::InvariantCulture)
    }

    $results += [pscustomobject]@{
        Run = $run
        ProgrammingSeconds = $programmingSeconds
        WallSeconds = [Math]::Round($timer.Elapsed.TotalSeconds, 1)
    }
}

$programmingTimes = @($results | Where-Object { $null -ne $_.ProgrammingSeconds } | ForEach-Object { $_.ProgrammingSeconds })
Write-Host '=== Summary ==='
$results | Format-Table -AutoSize | Out-String | Write-Host

if($programmingTimes.Count -gt 0)
{
    $min = ($programmingTimes | Measure-Object -Minimum).Minimum
    $max = ($programmingTimes | Measure-Object -Maximum).Maximum
    $avg = ($programmingTimes | Measure-Object -Average).Average
    Write-Host ("Programming time: min {0:N1} s, avg {1:N1} s, max {2:N1} s" -f $min, $avg, $max)
}

Write-Host 'Bootloader repeatability test passed.'
