[CmdletBinding()]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release',

    [string]$OutputDir,

    [switch]$KeepArtifacts
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$defsPath = Join-Path $repoRoot 'SignalSlinger\defs.h'
$buildScriptPath = Join-Path $repoRoot 'build-firmware.ps1'
$hexPath = Join-Path $repoRoot "SignalSlinger\$Configuration\SignalSlinger.hex"
$temporaryOutputDir = Join-Path $repoRoot 'tmp\firmware-hash-verification'
$targets = @('3.4', '3.5')

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

function Get-ActiveHardwareTarget {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DefsText
    )

    if($DefsText -match '(?m)^\s*#define HW_TARGET_3_4\s*$')
    {
        return '3.4'
    }

    if($DefsText -match '(?m)^\s*#define HW_TARGET_3_5\s*$')
    {
        return '3.5'
    }

    throw 'Could not determine the active hardware target from SignalSlinger/defs.h.'
}

function Get-SoftwareRevision {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DefsText
    )

    $match = [regex]::Match($DefsText, '(?m)^\s*#define SW_REVISION\s+"([^"]+)"\s*$')
    if(-not $match.Success)
    {
        throw 'Could not determine SW_REVISION from SignalSlinger/defs.h.'
    }

    return $match.Groups[1].Value
}

function Set-HardwareTargetContent {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DefsText,

        [Parameter(Mandatory = $true)]
        [ValidateSet('3.4', '3.5')]
        [string]$Target
    )

    $target34Line = if($Target -eq '3.4') { '#define HW_TARGET_3_4' } else { '// #define HW_TARGET_3_4' }
    $target35Line = if($Target -eq '3.5') { '#define HW_TARGET_3_5' } else { '// #define HW_TARGET_3_5' }

    $updated = [regex]::Replace($DefsText, '(?m)^\s*(?://\s*)?#define HW_TARGET_3_4\s*$', $target34Line)
    $updated = [regex]::Replace($updated, '(?m)^\s*(?://\s*)?#define HW_TARGET_3_5\s*$', $target35Line)

    return $updated
}

function Write-AsciiFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Content
    )

    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $Content, $encoding)
}

function Invoke-FirmwareBuild {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Debug', 'Release')]
        [string]$Configuration
    )

    & powershell -ExecutionPolicy Bypass -File $buildScriptPath -Configuration $Configuration
    if($LASTEXITCODE -ne 0)
    {
        throw "Build failed with exit code $LASTEXITCODE."
    }
}

Assert-PathExists -Path $defsPath -Description 'Firmware definitions file'
Assert-PathExists -Path $buildScriptPath -Description 'Firmware build script'

$originalDefsText = Get-Content -LiteralPath $defsPath -Raw
$originalTarget = Get-ActiveHardwareTarget -DefsText $originalDefsText
$softwareRevision = Get-SoftwareRevision -DefsText $originalDefsText
$hashResults = @()
$artifactsWereKept = $KeepArtifacts.IsPresent -or [bool]$OutputDir
$artifactDir = if($OutputDir) { $OutputDir } else { $temporaryOutputDir }
$lastBuiltTarget = $null

if($artifactsWereKept -or -not (Test-Path -LiteralPath $artifactDir))
{
    New-Item -ItemType Directory -Force -Path $artifactDir | Out-Null
}

try
{
    foreach($target in $targets)
    {
        $targetDefsText = Set-HardwareTargetContent -DefsText $originalDefsText -Target $target
        Write-Host "Building $Configuration firmware for hardware target $target..."
        Write-AsciiFile -Path $defsPath -Content $targetDefsText
        Invoke-FirmwareBuild -Configuration $Configuration
        $lastBuiltTarget = $target

        Assert-PathExists -Path $hexPath -Description "$Configuration HEX image"

        $artifactName = "SignalSlinger-v$softwareRevision-$target.hex"
        $artifactPath = Join-Path $artifactDir $artifactName
        Copy-Item -LiteralPath $hexPath -Destination $artifactPath -Force

        $hash = (Get-FileHash -LiteralPath $artifactPath -Algorithm SHA256).Hash
        $hashResults += [PSCustomObject]@{
            Target = $target
            SHA256 = $hash
            HexPath = $artifactPath
        }
    }
}
finally
{
    Write-AsciiFile -Path $defsPath -Content $originalDefsText

    if($lastBuiltTarget -and ($lastBuiltTarget -ne $originalTarget))
    {
        try
        {
            Write-Host "Restoring local $Configuration build output to hardware target $originalTarget..."
            Invoke-FirmwareBuild -Configuration $Configuration
        }
        catch
        {
            Write-Warning "Restoring the original $Configuration build output failed: $($_.Exception.Message)"
        }
    }

    if((-not $artifactsWereKept) -and (Test-Path -LiteralPath $artifactDir))
    {
        Remove-Item -LiteralPath $artifactDir -Recurse -Force
    }
}

Write-Host ''
Write-Host "Firmware hash verification for SW revision ${softwareRevision}:"
$hashResults | Format-Table -AutoSize
