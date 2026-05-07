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
$makePath = 'C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe'
$installedToolchainRoot = 'C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain'
$bundledToolchainRoot = Join-Path $repoRoot 'avr8-gnu-toolchain-win32_x86_64'
$bundledAvrGppPath = Join-Path $bundledToolchainRoot 'bin\avr-g++.exe'

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
Write-Host "Relocated firmware build completed: $elfPath"
