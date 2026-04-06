[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',

    [switch]$SkipBuild,

    [switch]$BuildOnly,

    [string]$Tool = 'atmelice',

    [string]$ToolSerial = 'J41800053674',

    [string]$Interface = 'UPDI',

    [string]$Device = 'avr128da28',

    [string]$Clock = '500kHz'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$projectRoot = Join-Path $repoRoot 'SignalSlinger'
$buildDir = Join-Path $projectRoot $Configuration
$elfPath = Join-Path $buildDir 'SignalSlinger.elf'
$makefilePath = Join-Path $buildDir 'Makefile'
$makePath = 'C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe'
$atprogramPath = 'C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe'
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
Assert-PathExists -Path $atprogramPath -Description 'Atmel Studio atprogram.exe'

if(-not $SkipBuild)
{
    $temporaryMakefilePath = $null

    try
    {
        $makeArguments = @('-C', $buildDir, 'all')

        if(Test-Path -LiteralPath $bundledAvrGppPath)
        {
            $temporaryMakefilePath = Join-Path $buildDir 'Makefile.codex-local'
            $makefileContent = Get-Content -LiteralPath $makefilePath -Raw
            $makefileContent = $makefileContent.Replace($installedToolchainRoot, $bundledToolchainRoot)
            Set-Content -LiteralPath $temporaryMakefilePath -Value $makefileContent -Encoding ASCII -NoNewline
            $makeArguments = @('-C', $buildDir, '-f', [System.IO.Path]::GetFileName($temporaryMakefilePath), 'all')
            Write-Host "Building $Configuration firmware with bundled AVR toolchain..."
        }
        else
        {
            Write-Host "Building $Configuration firmware with installed Atmel toolchain..."
        }

        & $makePath @makeArguments
        if($LASTEXITCODE -ne 0)
        {
            throw "Build failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        if($temporaryMakefilePath -and (Test-Path -LiteralPath $temporaryMakefilePath))
        {
            Remove-Item -LiteralPath $temporaryMakefilePath -Force
        }
    }
}

Assert-PathExists -Path $elfPath -Description "$Configuration ELF image"

if($BuildOnly)
{
    Write-Host "Build completed: $elfPath"
    exit 0
}

Write-Host "Flashing $Configuration firmware to $Device via $Tool ($ToolSerial) over $Interface..."
& $atprogramPath `
    -t $Tool `
    -s $ToolSerial `
    -i $Interface `
    -d $Device `
    -cl $Clock `
    program `
    -c `
    -fl `
    --verify `
    -f $elfPath

if($LASTEXITCODE -ne 0)
{
    throw "Flash failed with exit code $LASTEXITCODE."
}

Write-Host "Flash completed successfully: $elfPath"
