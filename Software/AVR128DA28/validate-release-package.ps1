[CmdletBinding()]
param(
    [string]$PackageDir = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot

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

function Resolve-PackageDirectory {
    if(-not [string]::IsNullOrWhiteSpace($PackageDir))
    {
        Assert-PathExists -Path $PackageDir -Description 'Release package folder'
        return (Resolve-Path -LiteralPath $PackageDir).Path
    }

    $packageRoot = Join-Path $repoRoot 'release-packages'
    Assert-PathExists -Path $packageRoot -Description 'Release packages folder'

    $latest = Get-ChildItem -LiteralPath $packageRoot -Directory |
        Where-Object { $_.Name -like 'SignalSlinger-*' } |
        Sort-Object LastWriteTimeUtc -Descending |
        Select-Object -First 1

    if($null -eq $latest)
    {
        throw "No SignalSlinger release package folders found in $packageRoot."
    }

    return $latest.FullName
}

function ConvertFrom-HexByte {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    return [Convert]::ToByte($Text, 16)
}

function Get-IntelHexBytes {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $bytesByAddress = @{}
    $upperAddress = 0

    foreach($rawLine in Get-Content -LiteralPath $Path)
    {
        $line = $rawLine.Trim()
        if($line.Length -eq 0)
        {
            continue
        }
        if(-not $line.StartsWith(':'))
        {
            throw "Invalid HEX line without ':' prefix in $Path`: $line"
        }

        $byteCount = ConvertFrom-HexByte $line.Substring(1, 2)
        $recordAddress = [Convert]::ToUInt16($line.Substring(3, 4), 16)
        $recordType = ConvertFrom-HexByte $line.Substring(7, 2)
        $expectedLength = 11 + ($byteCount * 2)
        if($line.Length -ne $expectedLength)
        {
            throw "Invalid HEX line length in $Path`: $line"
        }

        $sum = [int]$byteCount + (($recordAddress -shr 8) -band 0xFF) + ($recordAddress -band 0xFF) + [int]$recordType
        $data = [byte[]]::new($byteCount)
        for($i = 0; $i -lt $byteCount; $i++)
        {
            $data[$i] = ConvertFrom-HexByte $line.Substring(9 + ($i * 2), 2)
            $sum += $data[$i]
        }

        $checksum = ConvertFrom-HexByte $line.Substring(9 + ($byteCount * 2), 2)
        $sum += $checksum
        if(($sum -band 0xFF) -ne 0)
        {
            throw "HEX checksum failed in $Path`: $line"
        }

        switch($recordType)
        {
            0 {
                $absoluteAddress = $upperAddress + $recordAddress
                for($i = 0; $i -lt $byteCount; $i++)
                {
                    $bytesByAddress[$absoluteAddress + $i] = $data[$i]
                }
            }
            1 {
                return $bytesByAddress
            }
            2 {
                if($byteCount -ne 2)
                {
                    throw "Invalid extended segment address record length in $Path."
                }
                $upperAddress = ((([int]$data[0] -shl 8) -bor [int]$data[1]) -shl 4)
            }
            4 {
                if($byteCount -ne 2)
                {
                    throw "Invalid extended linear address record length in $Path."
                }
                $upperAddress = ((([int]$data[0] -shl 8) -bor [int]$data[1]) -shl 16)
            }
            3 {}
            5 {}
            default {
                throw "Unsupported HEX record type $recordType in $Path."
            }
        }
    }

    return $bytesByAddress
}

function Get-RequiredString {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Object,

        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    if(-not ($Object.PSObject.Properties.Name -contains $Name))
    {
        throw "Release info is missing '$Name'."
    }
    $value = [string]$Object.$Name
    if([string]::IsNullOrWhiteSpace($value))
    {
        throw "Release info field '$Name' is empty."
    }

    return $value
}

function Get-RequiredUInt {
    param(
        [Parameter(Mandatory = $true)]
        [object]$Object,

        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    if(-not ($Object.PSObject.Properties.Name -contains $Name))
    {
        throw "Release info is missing '$Name'."
    }

    return [uint32]$Object.$Name
}

function ConvertFrom-HexAddress {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    if($Text -notmatch '^0x[0-9A-Fa-f]+$')
    {
        throw "Invalid address: $Text"
    }

    return [uint32][Convert]::ToUInt32($Text.Substring(2), 16)
}

function Assert-BootloaderCapableVersion {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Version
    )

    $match = [regex]::Match($Version, '^(\d+)\.(\d+)\.(\d+)(?:[-+].*)?$')
    if(-not $match.Success)
    {
        throw "Release version must be a semantic version. Got '$Version'."
    }

    $major = [int]$match.Groups[1].Value
    if($major -lt 2)
    {
        throw "Release packages for bootloader-capable firmware require version 2.0.0 or newer. Got '$Version'."
    }
}

function Assert-Hash {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedHash
    )

    $actualHash = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash
    if($actualHash -ne $ExpectedHash)
    {
        throw ("Hash mismatch for {0}. Expected {1}, got {2}." -f (Split-Path -Leaf $Path), $ExpectedHash, $actualHash)
    }
}

function Read-ChecksumFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $checksums = @{}
    foreach($line in Get-Content -LiteralPath $Path)
    {
        if([string]::IsNullOrWhiteSpace($line))
        {
            continue
        }
        if($line -notmatch '^([0-9A-Fa-f]{64})\s+(.+)$')
        {
            throw "Invalid checksum line: $line"
        }
        $checksums[$Matches[2]] = $Matches[1].ToUpperInvariant()
    }

    return $checksums
}

function Get-AddressSummary {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$BytesByAddress
    )

    if($BytesByAddress.Count -eq 0)
    {
        throw 'HEX file has no data bytes.'
    }

    $addresses = @($BytesByAddress.Keys | Sort-Object { [int]$_ })
    return [pscustomobject]@{
        Count = $BytesByAddress.Count
        First = [uint32][int]($addresses | Select-Object -First 1)
        Last = [uint32][int]($addresses | Select-Object -Last 1)
    }
}

$resolvedPackageDir = Resolve-PackageDirectory
Write-Host "Validating release package: $resolvedPackageDir"

$releaseInfoPath = @(Get-ChildItem -LiteralPath $resolvedPackageDir -Filter 'SignalSlinger-Release-Info-*.json' -File)
if($releaseInfoPath.Count -ne 1)
{
    throw "Expected exactly one SignalSlinger-Release-Info JSON file, found $($releaseInfoPath.Count)."
}
$releaseInfoPath = $releaseInfoPath[0].FullName

$checksumsPath = @(Get-ChildItem -LiteralPath $resolvedPackageDir -Filter 'SignalSlinger-Checksums-*.txt' -File)
if($checksumsPath.Count -ne 1)
{
    throw "Expected exactly one SignalSlinger-Checksums file, found $($checksumsPath.Count)."
}
$checksumsPath = $checksumsPath[0].FullName

$releaseZipPath = @(Get-ChildItem -LiteralPath $resolvedPackageDir -Filter 'SignalSlinger-*-Release-Files.zip' -File)
if($releaseZipPath.Count -ne 1)
{
    throw "Expected exactly one SignalSlinger release zip file, found $($releaseZipPath.Count)."
}
$releaseZipPath = $releaseZipPath[0].FullName

$releaseInfo = Get-Content -LiteralPath $releaseInfoPath -Raw | ConvertFrom-Json
$format = Get-RequiredString -Object $releaseInfo -Name 'format'
if($format -ne 'signalslinger-release-info-v1')
{
    throw "Unsupported release-info format: $format"
}

$product = Get-RequiredString -Object $releaseInfo -Name 'product'
if($product -ne 'SignalSlinger')
{
    throw "Unexpected product: $product"
}

$version = Get-RequiredString -Object $releaseInfo -Name 'version'
Assert-BootloaderCapableVersion -Version $version
$board = Get-RequiredString -Object $releaseInfo -Name 'board'
$appStart = ConvertFrom-HexAddress (Get-RequiredString -Object $releaseInfo.serialSlinger -Name 'appStartAddress')
$pageBytes = Get-RequiredUInt -Object $releaseInfo.serialSlinger -Name 'pageBytes'
$flashBytes = Get-RequiredUInt -Object $releaseInfo.serialSlinger -Name 'flashBytes'
$protocolVersion = Get-RequiredUInt -Object $releaseInfo.serialSlinger -Name 'protocolVersion'
$updateBaud = Get-RequiredUInt -Object $releaseInfo.serialSlinger -Name 'updateBaud'
$appInfoCommand = Get-RequiredString -Object $releaseInfo.serialSlinger -Name 'appInfoCommand'
$appUpdateCommand = Get-RequiredString -Object $releaseInfo.serialSlinger -Name 'appUpdateCommand'
$bootloaderEntryCommand = Get-RequiredString -Object $releaseInfo.serialSlinger -Name 'bootloaderEntryCommand'

if($pageBytes -ne 512)
{
    throw "Unexpected update page size: $pageBytes"
}
if($flashBytes -ne 131072)
{
    throw "Unexpected flash size: $flashBytes"
}
if($protocolVersion -lt 1)
{
    throw "Unsupported protocol version: $protocolVersion"
}
if($updateBaud -ne 115200)
{
    throw "Unexpected update speed: $updateBaud"
}
if($appInfoCommand -ne 'INF')
{
    throw "Unexpected app info command: $appInfoCommand"
}
if($appUpdateCommand -ne 'UPD')
{
    throw "Unexpected app update command: $appUpdateCommand"
}
if($bootloaderEntryCommand -ne 'U')
{
    throw "Unexpected bootloader entry command: $bootloaderEntryCommand"
}

$checksums = Read-ChecksumFile -Path $checksumsPath
foreach($name in $checksums.Keys)
{
    $path = Join-Path $resolvedPackageDir $name
    Assert-PathExists -Path $path -Description "Checksum-listed file $name"
    Assert-Hash -Path $path -ExpectedHash $checksums[$name]
    Write-Host "OK checksum: $name"
}

$fileEntries = @($releaseInfo.files)
if($fileEntries.Count -lt 3)
{
    throw 'Release info does not list the expected package files.'
}

foreach($entry in $fileEntries)
{
    $fileName = Get-RequiredString -Object $entry -Name 'fileName'
    $kind = Get-RequiredString -Object $entry -Name 'kind'
    $sha256 = Get-RequiredString -Object $entry -Name 'sha256'
    $path = Join-Path $resolvedPackageDir $fileName
    Assert-PathExists -Path $path -Description "$kind file"
    Assert-Hash -Path $path -ExpectedHash $sha256
    if($checksums.ContainsKey($fileName) -and $checksums[$fileName] -ne $sha256.ToUpperInvariant())
    {
        throw "Checksum file and release info disagree for $fileName."
    }
    Write-Host "OK release info file: $fileName"
}

Add-Type -AssemblyName System.IO.Compression.FileSystem
$zipArchive = [System.IO.Compression.ZipFile]::OpenRead($releaseZipPath)
try
{
    $zipEntryNames = @($zipArchive.Entries | ForEach-Object { $_.Name })
    foreach($entry in $fileEntries)
    {
        $fileName = Get-RequiredString -Object $entry -Name 'fileName'
        if($zipEntryNames -notcontains $fileName)
        {
            throw "Release zip does not contain $fileName."
        }
    }
    foreach($requiredFile in $checksums.Keys)
    {
        if($zipEntryNames -notcontains $requiredFile)
        {
            throw "Release zip does not contain $requiredFile."
        }
    }
    foreach($requiredFile in @((Split-Path -Leaf $releaseInfoPath), (Split-Path -Leaf $checksumsPath)))
    {
        if($zipEntryNames -notcontains $requiredFile)
        {
            throw "Release zip does not contain $requiredFile."
        }
    }
}
finally
{
    if($null -ne $zipArchive)
    {
        $zipArchive.Dispose()
    }
}
Write-Host ("OK release zip: {0}" -f (Split-Path -Leaf $releaseZipPath))

$updateFile = Get-RequiredString -Object $releaseInfo.update -Name 'fileName'
$firstInstallFile = Get-RequiredString -Object $releaseInfo.firstInstall -Name 'fileName'
if($releaseInfo.workshopSetup.PSObject.Properties.Name -contains 'setupHelperFileName')
{
    $bootloaderFile = Get-RequiredString -Object $releaseInfo.workshopSetup -Name 'setupHelperFileName'
}
else
{
    $bootloaderFile = Get-RequiredString -Object $releaseInfo.workshopSetup -Name 'bootloaderFileName'
}

$updatePath = Join-Path $resolvedPackageDir $updateFile
$firstInstallPath = Join-Path $resolvedPackageDir $firstInstallFile
$bootloaderPath = Join-Path $resolvedPackageDir $bootloaderFile

foreach($path in @($updatePath, $firstInstallPath, $bootloaderPath))
{
    Assert-PathExists -Path $path -Description (Split-Path -Leaf $path)
}

$updateBytes = Get-IntelHexBytes -Path $updatePath
$firstInstallBytes = Get-IntelHexBytes -Path $firstInstallPath
$bootloaderBytes = Get-IntelHexBytes -Path $bootloaderPath

$updateSummary = Get-AddressSummary -BytesByAddress $updateBytes
$firstInstallSummary = Get-AddressSummary -BytesByAddress $firstInstallBytes
$bootloaderSummary = Get-AddressSummary -BytesByAddress $bootloaderBytes

if($updateSummary.First -ne $appStart)
{
    throw ("Update image starts at 0x{0:X}, expected 0x{1:X}." -f $updateSummary.First, $appStart)
}
if($updateSummary.Last -ge $flashBytes)
{
    throw ("Update image extends past flash: 0x{0:X}." -f $updateSummary.Last)
}
if($bootloaderSummary.First -ne 0)
{
    throw ("Bootloader helper image starts at 0x{0:X}, expected 0x0." -f $bootloaderSummary.First)
}
if($bootloaderSummary.Last -ge $appStart)
{
    throw ("Bootloader helper image reaches app space at 0x{0:X}." -f $bootloaderSummary.Last)
}
if($firstInstallSummary.First -ne 0)
{
    throw ("First-install image starts at 0x{0:X}, expected 0x0." -f $firstInstallSummary.First)
}
if($firstInstallSummary.Last -lt $updateSummary.Last)
{
    throw 'First-install image does not include the full update image.'
}

if([uint32]$releaseInfo.update.bytesInImage -ne $updateSummary.Count)
{
    throw 'Update byte count in release info does not match the update file.'
}
if([uint32]$releaseInfo.firstInstall.bytesInImage -ne $firstInstallSummary.Count)
{
    throw 'First-install byte count in release info does not match the first-install file.'
}

Write-Host ''
Write-Host ("Release package OK: SignalSlinger {0}, {1}" -f $version, $board)
Write-Host ("Update image: 0x{0:X}..0x{1:X} ({2} bytes)" -f $updateSummary.First, $updateSummary.Last, $updateSummary.Count)
Write-Host ("First-install image: 0x{0:X}..0x{1:X} ({2} bytes)" -f $firstInstallSummary.First, $firstInstallSummary.Last, $firstInstallSummary.Count)
Write-Host ("Bootloader helper image: 0x{0:X}..0x{1:X} ({2} bytes)" -f $bootloaderSummary.First, $bootloaderSummary.Last, $bootloaderSummary.Count)
