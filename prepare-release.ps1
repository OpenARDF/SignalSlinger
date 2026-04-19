[CmdletBinding()]
param(
    [string]$Version,

    [switch]$SkipReadmeUpdate,

    [switch]$SkipFinalDefaultBuild
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$repoRoot = $PSScriptRoot
$softwareRoot = Join-Path $repoRoot 'Software\AVR128DA28'
$projectRoot = Join-Path $softwareRoot 'SignalSlinger'
$releaseDir = Join-Path $projectRoot 'Release'
$defsPath = Join-Path $projectRoot 'defs.h'
$readmePath = Join-Path $repoRoot 'README.md'
$releaseHexPath = Join-Path $releaseDir 'SignalSlinger.hex'
$makePath = 'C:\Program Files (x86)\Atmel\Studio\7.0\shellutils\make.exe'

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

function Read-TextFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    return [System.IO.File]::ReadAllText($Path)
}

function Write-TextFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Content
    )

    [System.IO.File]::WriteAllText($Path, $Content, $utf8NoBom)
}

function Get-ReleaseVersionTag {
    param(
        [Parameter(Mandatory = $true)]
        [string]$InputVersion
    )

    if($InputVersion.StartsWith('v'))
    {
        return $InputVersion
    }

    return "v$InputVersion"
}

function Set-HardwareTargetContent {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DefsText,

        [Parameter(Mandatory = $true)]
        [ValidateSet('3.4', '3.5')]
        [string]$HardwareVersion
    )

    switch($HardwareVersion)
    {
        '3.4' {
            $updated = $DefsText `
                -replace '(?m)^// #define HW_TARGET_3_4$','#define HW_TARGET_3_4' `
                -replace '(?m)^#define HW_TARGET_3_5$','// #define HW_TARGET_3_5'
        }
        '3.5' {
            $updated = $DefsText `
                -replace '(?m)^#define HW_TARGET_3_4$','// #define HW_TARGET_3_4' `
                -replace '(?m)^// #define HW_TARGET_3_5$','#define HW_TARGET_3_5'
        }
    }

    return $updated
}

function Invoke-ReleaseMake {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Targets
    )

    & $makePath -C $releaseDir @Targets
    if($LASTEXITCODE -ne 0)
    {
        throw "Release build command failed with exit code $LASTEXITCODE."
    }
}

function Build-ReleaseAsset {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('3.4', '3.5')]
        [string]$HardwareVersion,

        [Parameter(Mandatory = $true)]
        [string]$VersionTag
    )

    Write-Host "Building Release firmware for hardware $HardwareVersion..."

    $defsText = Read-TextFile -Path $defsPath
    $updatedDefs = Set-HardwareTargetContent -DefsText $defsText -HardwareVersion $HardwareVersion
    if($updatedDefs -ne $defsText)
    {
        Write-TextFile -Path $defsPath -Content $updatedDefs
    }

    Invoke-ReleaseMake -Targets @('clean')
    Invoke-ReleaseMake -Targets @('all')

    $assetPath = Join-Path $releaseDir "SignalSlinger-$VersionTag-$HardwareVersion.hex"
    Copy-Item -LiteralPath $releaseHexPath -Destination $assetPath -Force
    Write-Host "Wrote release asset: $assetPath"
}

function Update-ReadmeVersionReferences {
    param(
        [Parameter(Mandatory = $true)]
        [string]$VersionTag
    )

    if($SkipReadmeUpdate)
    {
        Write-Host 'Skipping README update.'
        return
    }

    $readmeText = Read-TextFile -Path $readmePath
    $updatedReadme = $readmeText `
        -replace 'Firmware version: v[0-9A-Za-z.\-]+',"Firmware version: $VersionTag" `
        -replace 'SignalSlinger-v[0-9A-Za-z.\-]+-3\.5\.hex',"SignalSlinger-$VersionTag-3.5.hex" `
        -replace 'SignalSlinger-v[0-9A-Za-z.\-]+-3\.4\.hex',"SignalSlinger-$VersionTag-3.4.hex"

    if($updatedReadme -ne $readmeText)
    {
        Write-TextFile -Path $readmePath -Content $updatedReadme
        Write-Host "Updated README release references to $VersionTag."
    }
    else
    {
        Write-Host 'README already matched the requested release version.'
    }
}

Assert-PathExists -Path $softwareRoot -Description 'Firmware root'
Assert-PathExists -Path $projectRoot -Description 'Firmware project root'
Assert-PathExists -Path $releaseDir -Description 'Release build directory'
Assert-PathExists -Path $defsPath -Description 'defs.h'
Assert-PathExists -Path $readmePath -Description 'README.md'
Assert-PathExists -Path $makePath -Description 'Atmel Studio make.exe'

$originalDefs = Read-TextFile -Path $defsPath

if([string]::IsNullOrWhiteSpace($Version))
{
    $versionMatch = [regex]::Match($originalDefs, '(?m)^#define SW_REVISION "([^"]+)"$')
    if(-not $versionMatch.Success)
    {
        throw "Unable to determine SW_REVISION from $defsPath"
    }

    $Version = $versionMatch.Groups[1].Value
}

$versionTag = Get-ReleaseVersionTag -InputVersion $Version

try
{
    Build-ReleaseAsset -HardwareVersion '3.4' -VersionTag $versionTag
    Build-ReleaseAsset -HardwareVersion '3.5' -VersionTag $versionTag
    Update-ReadmeVersionReferences -VersionTag $versionTag
}
finally
{
    $currentDefs = Read-TextFile -Path $defsPath
    if($currentDefs -ne $originalDefs)
    {
        Write-TextFile -Path $defsPath -Content $originalDefs
        Write-Host 'Restored defs.h to its original hardware-target setting.'
    }
}

if(-not $SkipFinalDefaultBuild)
{
    Write-Host 'Rebuilding the default Release output for the restored hardware target...'
    Invoke-ReleaseMake -Targets @('clean')
    Invoke-ReleaseMake -Targets @('all')
}

$asset34 = Join-Path $releaseDir "SignalSlinger-$versionTag-3.4.hex"
$asset35 = Join-Path $releaseDir "SignalSlinger-$versionTag-3.5.hex"

Write-Host ''
Write-Host "Prepared release assets:"
Write-Host "  $asset34"
Write-Host "  $asset35"
