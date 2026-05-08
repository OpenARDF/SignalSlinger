[CmdletBinding()]
param(
    [ValidateSet('Release')]
    [string]$Configuration = 'Release',

    [string]$OutputRoot = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$defsPath = Join-Path $repoRoot 'SignalSlinger\defs.h'
$packageScript = Join-Path $repoRoot 'build-release-package.ps1'

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

    if($DefsText -match '(?m)^\s*#define\s+HW_TARGET_3_4\s*$')
    {
        return '3.4'
    }
    if($DefsText -match '(?m)^\s*#define\s+HW_TARGET_3_5\s*$')
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

    $match = [regex]::Match($DefsText, '(?m)^\s*#define\s+SW_REVISION\s+"([^"]+)"\s*$')
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

    $updated = [regex]::Replace($DefsText, '(?m)^\s*(?://\s*)?#define\s+HW_TARGET_3_4\s*$', $target34Line)
    $updated = [regex]::Replace($updated, '(?m)^\s*(?://\s*)?#define\s+HW_TARGET_3_5\s*$', $target35Line)

    return $updated
}

function Write-TextFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Content
    )

    $encoding = [System.Text.UTF8Encoding]::new($false)
    [System.IO.File]::WriteAllText($Path, $Content, $encoding)
}

Assert-PathExists -Path $defsPath -Description 'SignalSlinger definitions file'
Assert-PathExists -Path $packageScript -Description 'Release package builder'

$originalDefsText = Get-Content -LiteralPath $defsPath -Raw
$originalTarget = Get-ActiveHardwareTarget -DefsText $originalDefsText
$softwareRevision = Get-SoftwareRevision -DefsText $originalDefsText
$safeVersion = $softwareRevision -replace '[^A-Za-z0-9._-]', '-'

if([string]::IsNullOrWhiteSpace($OutputRoot))
{
    $OutputRoot = Join-Path $repoRoot 'release-packages'
}

$builtPackages = [System.Collections.Generic.List[object]]::new()
$lastBuiltTarget = $null

try
{
    foreach($target in @('3.4', '3.5'))
    {
        Write-Host ("Building SignalSlinger release package for HW-{0}..." -f $target)
        $targetDefsText = Set-HardwareTargetContent -DefsText $originalDefsText -Target $target
        Write-TextFile -Path $defsPath -Content $targetDefsText

        $outputDir = Join-Path $OutputRoot ("SignalSlinger-{0}-HW-{1}" -f $safeVersion, $target)
        & powershell -ExecutionPolicy Bypass -File $packageScript -Configuration $Configuration -OutputDir $outputDir
        if($LASTEXITCODE -ne 0)
        {
            throw "Release package build failed for HW-$target with exit code $LASTEXITCODE."
        }

        $lastBuiltTarget = $target
        $builtPackages.Add([pscustomobject]@{
            Hardware = "HW-$target"
            OutputDir = $outputDir
        })
    }
}
finally
{
    Write-TextFile -Path $defsPath -Content $originalDefsText

    if($lastBuiltTarget -and $lastBuiltTarget -ne $originalTarget)
    {
        try
        {
            Write-Host ("Restoring local relocated build output to HW-{0}..." -f $originalTarget)
            & powershell -ExecutionPolicy Bypass -File (Join-Path $repoRoot 'build-relocated-firmware.ps1') -Configuration $Configuration
            if($LASTEXITCODE -ne 0)
            {
                Write-Warning "Restoring local relocated build output failed with exit code $LASTEXITCODE."
            }
        }
        catch
        {
            Write-Warning "Restoring local relocated build output failed: $($_.Exception.Message)"
        }
    }
}

Write-Host ''
Write-Host 'Release packages created:'
$builtPackages | Format-Table -AutoSize | Out-String | Write-Host
Write-Host 'For GitHub releases, upload each HW-specific unzipped update HEX and each matching release ZIP.'
