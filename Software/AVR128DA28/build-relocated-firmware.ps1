[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release',

    [ValidatePattern('^0x[0-9A-Fa-f]+$')]
    [string]$ApplicationStart = '0x4000'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$projectRoot = Join-Path $repoRoot 'SignalSlinger'
$buildDir = Join-Path $projectRoot $Configuration
$buildSrcDir = Join-Path $buildDir 'src'
$makefilePath = Join-Path $buildDir 'Makefile'
$elfPath = Join-Path $buildDir 'SignalSlinger.elf'
$hexPath = Join-Path $buildDir 'SignalSlinger.hex'
$mapPath = Join-Path $buildDir 'SignalSlinger.map'
$makePath = 'C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe'
$installedToolchainRoot = 'C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain'
$bundledToolchainRoot = Join-Path $repoRoot 'avr8-gnu-toolchain-win32_x86_64'
$bundledAvrGppPath = Join-Path $bundledToolchainRoot 'bin\avr-g++.exe'
$relocatedOutputNames = @(
    'SignalSlinger.elf',
    'SignalSlinger.hex',
    'SignalSlinger.eep',
    'SignalSlinger.lss',
    'SignalSlinger.map',
    'SignalSlinger.srec',
    'SignalSlinger.usersignatures'
)

function Assert-PathExists {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    if(-not (Test-Path -LiteralPath $Path))
    {
        throw "$Description not found: $Path"
    }
}

Assert-PathExists -Path $projectRoot -Description 'Project root'
Assert-PathExists -Path $buildDir -Description 'Build directory'
Assert-PathExists -Path $makefilePath -Description "$Configuration Makefile"
Assert-PathExists -Path $makePath -Description 'Atmel Studio make.exe'

$resolvedBuildDir = [System.IO.Path]::GetFullPath($buildDir)

if(-not (Test-Path -LiteralPath $buildSrcDir))
{
    New-Item -ItemType Directory -Path $buildSrcDir | Out-Null
}

$temporaryMakefilePath = Join-Path $buildDir 'Makefile.codex-relocated'

try
{
    $makefileContent = Get-Content -LiteralPath $makefilePath -Raw
    $linkerMapFlag = '-Wl,-Map="SignalSlinger.map"'

    if($makefileContent -notlike "*$linkerMapFlag*")
    {
        throw "Could not find expected linker map flag in $makefilePath."
    }

    $makefileContent = $makefileContent.Replace(
        $linkerMapFlag,
        "$linkerMapFlag -Wl,--section-start=.text=$ApplicationStart"
    )

    if(Test-Path -LiteralPath $bundledAvrGppPath)
    {
        $makefileContent = $makefileContent.Replace($installedToolchainRoot, $bundledToolchainRoot)
        Write-Host "Building relocated $Configuration firmware with bundled AVR toolchain at .text=$ApplicationStart..."
    }
    else
    {
        Write-Host "Building relocated $Configuration firmware with installed Atmel toolchain at .text=$ApplicationStart..."
    }

    Set-Content -LiteralPath $temporaryMakefilePath -Value $makefileContent -Encoding ASCII -NoNewline

    foreach($outputName in $relocatedOutputNames)
    {
        $outputPath = Join-Path $buildDir $outputName
        $resolvedOutputPath = [System.IO.Path]::GetFullPath($outputPath)
        if(-not $resolvedOutputPath.StartsWith($resolvedBuildDir, [System.StringComparison]::OrdinalIgnoreCase))
        {
            throw ("Refusing to remove build output outside {0}: {1}" -f $resolvedBuildDir, $resolvedOutputPath)
        }
        if(Test-Path -LiteralPath $outputPath)
        {
            Remove-Item -LiteralPath $outputPath -Force
        }
    }
    Write-Host 'Removed previous final firmware outputs to force a relocated relink.'

    & $makePath -C $buildDir -f ([System.IO.Path]::GetFileName($temporaryMakefilePath)) all
    if($LASTEXITCODE -ne 0)
    {
        throw "Relocated firmware build failed with exit code $LASTEXITCODE."
    }
}
finally
{
    if(Test-Path -LiteralPath $temporaryMakefilePath)
    {
        Remove-Item -LiteralPath $temporaryMakefilePath -Force
    }
}

Assert-PathExists -Path $elfPath -Description "$Configuration relocated ELF image"
Assert-PathExists -Path $hexPath -Description "$Configuration relocated HEX image"
Assert-PathExists -Path $mapPath -Description "$Configuration relocated map"

$expectedTextStart = [Convert]::ToUInt32($ApplicationStart.Substring(2), 16)
$mapText = Get-Content -LiteralPath $mapPath -Raw
$textMatch = [regex]::Match($mapText, '(?m)^\.text\s+0x([0-9A-Fa-f]+)\s+0x([0-9A-Fa-f]+)')
if(-not $textMatch.Success)
{
    throw "Could not find .text section start in $mapPath."
}

$actualTextStart = [Convert]::ToUInt32($textMatch.Groups[1].Value, 16)
$textSize = [Convert]::ToUInt32($textMatch.Groups[2].Value, 16)
if($actualTextStart -ne $expectedTextStart)
{
    throw ("Relocated build verification failed: .text starts at 0x{0:X}, expected {1}." -f $actualTextStart, $ApplicationStart)
}

Write-Host "Relocated firmware build completed: $elfPath"
Write-Host ("Verified relocated .text start: 0x{0:X}, size 0x{1:X}." -f $actualTextStart, $textSize)
