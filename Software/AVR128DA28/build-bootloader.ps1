[CmdletBinding()]
param(
    [ValidateSet('Release')]
    [string]$Configuration = 'Release'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$bootloaderRoot = Join-Path $repoRoot 'bootloader'
$buildDir = Join-Path $bootloaderRoot $Configuration
$makefilePath = Join-Path $buildDir 'Makefile'
$elfPath = Join-Path $buildDir 'SignalSlingerBootloader.elf'
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

Assert-PathExists -Path $bootloaderRoot -Description 'Bootloader root'
Assert-PathExists -Path $buildDir -Description "$Configuration bootloader build directory"
Assert-PathExists -Path $makefilePath -Description "$Configuration bootloader Makefile"
Assert-PathExists -Path $makePath -Description 'Atmel Studio make.exe'

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
        Write-Host "Building $Configuration bootloader with bundled AVR toolchain..."
    }
    else
    {
        Write-Host "Building $Configuration bootloader with installed Atmel toolchain..."
    }

    & $makePath @makeArguments
    if($LASTEXITCODE -ne 0)
    {
        throw "Bootloader build failed with exit code $LASTEXITCODE."
    }
}
finally
{
    if($temporaryMakefilePath -and (Test-Path -LiteralPath $temporaryMakefilePath))
    {
        Remove-Item -LiteralPath $temporaryMakefilePath -Force
    }
}

Assert-PathExists -Path $elfPath -Description "$Configuration bootloader ELF image"
Write-Host "Bootloader build completed: $elfPath"
