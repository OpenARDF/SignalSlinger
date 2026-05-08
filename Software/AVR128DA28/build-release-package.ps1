[CmdletBinding()]
param(
    [ValidateSet('Release')]
    [string]$Configuration = 'Release',

    [string]$OutputDir = '',

    [switch]$SkipBuild,

    [switch]$SkipValidate
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$repoRoot = $PSScriptRoot
$defsPath = Join-Path $repoRoot 'SignalSlinger\defs.h'
$bootConfigPath = Join-Path $repoRoot 'bootloader\include\bootloader_config.h'
$bootloaderHexPath = Join-Path $repoRoot 'bootloader\Release\SignalSlingerBootloader.hex'
$applicationHexPath = Join-Path $repoRoot 'SignalSlinger\Release\SignalSlinger.hex'
$applicationMapPath = Join-Path $repoRoot 'SignalSlinger\Release\SignalSlinger.map'
$bootloaderMapPath = Join-Path $repoRoot 'bootloader\Release\SignalSlingerBootloader.map'

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

function Get-DefineString {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text,

        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    $match = [regex]::Match($Text, "(?m)^\s*#define\s+$Name\s+`"([^`"]+)`"\s*$")
    if(-not $match.Success)
    {
        throw "Could not find #define $Name."
    }

    return $match.Groups[1].Value
}

function Get-DefineUInt {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text,

        [Parameter(Mandatory = $true)]
        [string]$Name
    )

    $match = [regex]::Match($Text, "(?m)^\s*#define\s+$Name\s+(.+?)\s*$")
    if(-not $match.Success)
    {
        throw "Could not find #define $Name."
    }

    $valueText = $match.Groups[1].Value.Trim()
    $valueText = $valueText -replace '[uUlL]+$', ''
    if($valueText -match '^0x[0-9A-Fa-f]+$')
    {
        return [uint32][Convert]::ToUInt32($valueText.Substring(2), 16)
    }
    if($valueText -match '^\d+$')
    {
        return [uint32]$valueText
    }

    throw "Could not parse numeric #define $Name value: $valueText"
}

function Get-ActiveHardwareName {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DefsText
    )

    if($DefsText -match '(?m)^\s*#define\s+HW_TARGET_3_5\s*$')
    {
        return 'Board-3.5'
    }
    if($DefsText -match '(?m)^\s*#define\s+HW_TARGET_3_4\s*$')
    {
        return 'Board-3.4'
    }

    throw 'Could not determine the active SignalSlinger board version.'
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

function New-IntelHexRecord {
    param(
        [Parameter(Mandatory = $true)]
        [byte]$RecordType,

        [Parameter(Mandatory = $true)]
        [UInt16]$Address,

        [byte[]]$Data = [byte[]]::new(0)
    )

    $sum = $Data.Length + (($Address -shr 8) -band 0xFF) + ($Address -band 0xFF) + $RecordType
    $hex = ':' + ('{0:X2}{1:X4}{2:X2}' -f $Data.Length, $Address, $RecordType)
    foreach($byte in $Data)
    {
        $sum += $byte
        $hex += ('{0:X2}' -f $byte)
    }
    $checksum = ((-$sum) -band 0xFF)
    return $hex + ('{0:X2}' -f $checksum)
}

function Write-IntelHex {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$BytesByAddress,

        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $parent = Split-Path -Parent $Path
    if(-not [string]::IsNullOrWhiteSpace($parent))
    {
        New-Item -ItemType Directory -Force -Path $parent | Out-Null
    }

    $lines = [System.Collections.Generic.List[string]]::new()
    $currentUpper = -1
    $addresses = @($BytesByAddress.Keys | Sort-Object { [int]$_ })
    $index = 0

    while($index -lt $addresses.Count)
    {
        $startAddress = [int]$addresses[$index]
        $upper = ($startAddress -shr 16) -band 0xFFFF
        if($upper -ne $currentUpper)
        {
            $lines.Add((New-IntelHexRecord -RecordType 4 -Address 0 -Data ([byte[]]@([byte](($upper -shr 8) -band 0xFF), [byte]($upper -band 0xFF)))))
            $currentUpper = $upper
        }

        $chunkStart = $startAddress
        $chunk = [System.Collections.Generic.List[byte]]::new()
        while($index -lt $addresses.Count -and $chunk.Count -lt 16)
        {
            $address = [int]$addresses[$index]
            if($address -ne ($chunkStart + $chunk.Count) -or (($address -shr 16) -band 0xFFFF) -ne $currentUpper)
            {
                break
            }
            $chunk.Add([byte]$BytesByAddress[$address])
            $index++
        }

        $lines.Add((New-IntelHexRecord -RecordType 0 -Address ([UInt16]($chunkStart -band 0xFFFF)) -Data $chunk.ToArray()))
    }

    $lines.Add(':00000001FF')
    Set-Content -LiteralPath $Path -Value $lines -Encoding ASCII
}

function Merge-HexFiles {
    param(
        [Parameter(Mandatory = $true)]
        [string]$BootloaderPath,

        [Parameter(Mandatory = $true)]
        [string]$ApplicationPath,

        [Parameter(Mandatory = $true)]
        [string]$OutputPath
    )

    $bootBytes = Get-IntelHexBytes -Path $BootloaderPath
    $appBytes = Get-IntelHexBytes -Path $ApplicationPath
    $combined = @{}

    foreach($address in $bootBytes.Keys)
    {
        $combined[[int]$address] = [byte]$bootBytes[$address]
    }

    foreach($address in $appBytes.Keys)
    {
        $intAddress = [int]$address
        if($combined.ContainsKey($intAddress) -and [byte]$combined[$intAddress] -ne [byte]$appBytes[$address])
        {
            throw ("HEX overlap at 0x{0:X} between bootloader and app." -f $intAddress)
        }
        $combined[$intAddress] = [byte]$appBytes[$address]
    }

    Write-IntelHex -BytesByAddress $combined -Path $OutputPath
    return [pscustomobject]@{
        BootloaderBytes = $bootBytes.Count
        ApplicationBytes = $appBytes.Count
        CombinedBytes = $combined.Count
    }
}

function Assert-TextStart {
    param(
        [Parameter(Mandatory = $true)]
        [string]$MapPath,

        [Parameter(Mandatory = $true)]
        [uint32]$ExpectedStart,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    $mapText = Get-Content -LiteralPath $MapPath -Raw
    $match = [regex]::Match($mapText, '(?m)^\.text\s+0x([0-9A-Fa-f]+)\s+0x([0-9A-Fa-f]+)')
    if(-not $match.Success)
    {
        throw "Could not find .text in $MapPath."
    }

    $actualStart = [uint32][Convert]::ToUInt32($match.Groups[1].Value, 16)
    $size = [uint32][Convert]::ToUInt32($match.Groups[2].Value, 16)
    if($actualStart -ne $ExpectedStart)
    {
        throw ("{0} starts at 0x{1:X}, expected 0x{2:X}." -f $Description, $actualStart, $ExpectedStart)
    }

    return [pscustomobject]@{
        Start = $actualStart
        Size = $size
    }
}

function Copy-PackageFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$SourcePath,

        [Parameter(Mandatory = $true)]
        [string]$DestinationPath,

        [Parameter(Mandatory = $true)]
        [string]$Purpose,

        [Parameter(Mandatory = $true)]
        [string]$Kind
    )

    $resolvedSource = [System.IO.Path]::GetFullPath($SourcePath)
    $resolvedDestination = [System.IO.Path]::GetFullPath($DestinationPath)
    if(-not $resolvedSource.Equals($resolvedDestination, [System.StringComparison]::OrdinalIgnoreCase))
    {
        Copy-Item -LiteralPath $SourcePath -Destination $DestinationPath -Force
    }
    $hash = (Get-FileHash -LiteralPath $DestinationPath -Algorithm SHA256).Hash
    $item = Get-Item -LiteralPath $DestinationPath

    return [pscustomobject]@{
        fileName = $item.Name
        kind = $Kind
        purpose = $Purpose
        sizeBytes = $item.Length
        sha256 = $hash
    }
}

Assert-PathExists -Path $defsPath -Description 'SignalSlinger definitions'
Assert-PathExists -Path $bootConfigPath -Description 'Bootloader configuration'

if(-not $SkipBuild)
{
    & (Join-Path $repoRoot 'build-bootloader.ps1') -Configuration $Configuration
    if($LASTEXITCODE -ne 0)
    {
        throw "Bootloader build failed with exit code $LASTEXITCODE."
    }

    & (Join-Path $repoRoot 'build-relocated-firmware.ps1') -Configuration $Configuration
    if($LASTEXITCODE -ne 0)
    {
        throw "SignalSlinger update build failed with exit code $LASTEXITCODE."
    }
}

Assert-PathExists -Path $bootloaderHexPath -Description 'SignalSlinger bootloader HEX'
Assert-PathExists -Path $applicationHexPath -Description 'SignalSlinger update HEX'
Assert-PathExists -Path $applicationMapPath -Description 'SignalSlinger update map'
Assert-PathExists -Path $bootloaderMapPath -Description 'SignalSlinger bootloader map'

$defsText = Get-Content -LiteralPath $defsPath -Raw
$bootConfigText = Get-Content -LiteralPath $bootConfigPath -Raw

$softwareVersion = Get-DefineString -Text $defsText -Name 'SW_REVISION'
$productName = Get-DefineString -Text $defsText -Name 'PRODUCT_NAME_SHORT'
$boardName = Get-ActiveHardwareName -DefsText $defsText
$bootloaderVersion = Get-DefineString -Text $bootConfigText -Name 'SIGNALSLINGER_BOOTLOADER_VERSION'
$protocolVersion = Get-DefineUInt -Text $bootConfigText -Name 'SIGNALSLINGER_BOOT_PROTOCOL_VERSION'
$bootPages = Get-DefineUInt -Text $bootConfigText -Name 'SIGNALSLINGER_BOOT_SECTION_PAGES'
$pageBytes = Get-DefineUInt -Text $bootConfigText -Name 'SIGNALSLINGER_FLASH_PAGE_BYTES'
$flashBytes = Get-DefineUInt -Text $bootConfigText -Name 'SIGNALSLINGER_FLASH_BYTES'
$bootBaud = Get-DefineUInt -Text $bootConfigText -Name 'SIGNALSLINGER_BOOT_USART_BAUD'
$appStart = $bootPages * $pageBytes
$appBaud = 9600

$appText = Assert-TextStart -MapPath $applicationMapPath -ExpectedStart $appStart -Description 'SignalSlinger update image'
$bootText = Assert-TextStart -MapPath $bootloaderMapPath -ExpectedStart 0 -Description 'SignalSlinger bootloader image'

if([string]::IsNullOrWhiteSpace($OutputDir))
{
    $safeVersion = $softwareVersion -replace '[^A-Za-z0-9._-]', '-'
    $OutputDir = Join-Path $repoRoot (Join-Path 'release-packages' "SignalSlinger-$safeVersion-$boardName")
}
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$generatedPackagePatterns = @(
    'SignalSlinger-*.hex',
    'SignalSlinger-Release-Info-*.json',
    'SignalSlinger-Checksums-*.txt',
    'README-SignalSlinger-*.txt'
)
foreach($pattern in $generatedPackagePatterns)
{
    Get-ChildItem -LiteralPath $OutputDir -File -Filter $pattern -ErrorAction SilentlyContinue |
        Remove-Item -Force
}

$friendlyVersion = "v$softwareVersion"
$updateFile = "SignalSlinger-Update-$friendlyVersion-$boardName.hex"
$firstInstallFile = "SignalSlinger-First-Install-$friendlyVersion-$boardName.hex"
$bootloaderFile = "SignalSlinger-Setup-Helper-$bootloaderVersion.hex"
$releaseInfoFile = "SignalSlinger-Release-Info-$friendlyVersion-$boardName.json"
$checksumsFile = "SignalSlinger-Checksums-$friendlyVersion-$boardName.txt"
$readmeFile = "README-SignalSlinger-$friendlyVersion.txt"

$updatePath = Join-Path $OutputDir $updateFile
$firstInstallPath = Join-Path $OutputDir $firstInstallFile
$bootloaderPath = Join-Path $OutputDir $bootloaderFile
$releaseInfoPath = Join-Path $OutputDir $releaseInfoFile
$checksumsPath = Join-Path $OutputDir $checksumsFile
$readmePath = Join-Path $OutputDir $readmeFile

$mergeSummary = Merge-HexFiles -BootloaderPath $bootloaderHexPath -ApplicationPath $applicationHexPath -OutputPath $firstInstallPath

$files = @()
$files += Copy-PackageFile -SourcePath $applicationHexPath -DestinationPath $updatePath -Kind 'update' -Purpose 'For SerialSlinger to update a SignalSlinger that already supports software updates.'
$files += Copy-PackageFile -SourcePath $firstInstallPath -DestinationPath $firstInstallPath -Kind 'first-install' -Purpose 'For workshop setup of a new board using a programmer.'
$files += Copy-PackageFile -SourcePath $bootloaderHexPath -DestinationPath $bootloaderPath -Kind 'setup-helper' -Purpose 'Used by workshop setup tools when installing update support.'

$gitCommit = ''
try
{
    $gitCommit = (& git -C $repoRoot rev-parse HEAD 2>$null).Trim()
}
catch
{
    $gitCommit = 'unknown'
}

$releaseInfo = [pscustomobject]@{
    format = 'signalslinger-release-info-v1'
    product = $productName
    version = $softwareVersion
    board = $boardName
    generatedUtc = [DateTime]::UtcNow.ToString('o')
    gitCommit = $gitCommit
    update = [pscustomobject]@{
        fileName = $updateFile
        startAddress = ('0x{0:X}' -f $appStart)
        bytesInImage = $mergeSummary.ApplicationBytes
    }
    firstInstall = [pscustomobject]@{
        fileName = $firstInstallFile
        bytesInImage = $mergeSummary.CombinedBytes
    }
    serialSlinger = [pscustomobject]@{
        appBaud = $appBaud
        updateBaud = $bootBaud
        pageBytes = $pageBytes
        protocolVersion = $protocolVersion
        bootloaderVersion = $bootloaderVersion
        appStartAddress = ('0x{0:X}' -f $appStart)
        flashBytes = $flashBytes
    }
    workshopSetup = [pscustomobject]@{
        setupHelperFileName = $bootloaderFile
        bootSectionPages = $bootPages
        fuseBootSize = '0x20'
        fuseCodeSize = '0x00'
    }
    files = $files
}

$releaseInfo | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $releaseInfoPath -Encoding UTF8
$files = @($files + (Copy-PackageFile -SourcePath $releaseInfoPath -DestinationPath $releaseInfoPath -Kind 'release-info' -Purpose 'Information SerialSlinger reads to choose and verify files.'))

$readme = @"
SignalSlinger $friendlyVersion

This folder is intended for release uploads and for the SerialSlinger app.

Files:

- $updateFile
  SerialSlinger uses this file to update a SignalSlinger that already supports software updates.

- $firstInstallFile
  Workshop setup tools use this file when preparing a new board with a programmer.

- $bootloaderFile
  Workshop setup tools use this helper file when adding software-update support to a new board.

- $releaseInfoFile
  SerialSlinger reads this information automatically. Most users do not need to open it.

- $checksumsFile
  Optional file-integrity checks.

Board: $boardName
Update speed: $bootBaud baud
Update verification: page-by-page checks built into the SignalSlinger update process

For normal use, open SerialSlinger and let it choose the right file.
"@
$readme | Set-Content -LiteralPath $readmePath -Encoding UTF8
$files = @($files + (Copy-PackageFile -SourcePath $readmePath -DestinationPath $readmePath -Kind 'readme' -Purpose 'Plain-language notes for the release folder.'))

$checksumLines = [System.Collections.Generic.List[string]]::new()
foreach($file in $files)
{
    $checksumLines.Add(('{0}  {1}' -f $file.sha256, $file.fileName))
}
$checksumLines | Set-Content -LiteralPath $checksumsPath -Encoding ASCII

Write-Host "Release package created: $OutputDir"
Write-Host ("Update file: {0}" -f $updateFile)
Write-Host ("First-install file: {0}" -f $firstInstallFile)
Write-Host ("Release info: {0}" -f $releaseInfoFile)
Write-Host ("App .text start verified: 0x{0:X}, size 0x{1:X}" -f $appText.Start, $appText.Size)
Write-Host ("Bootloader .text start verified: 0x{0:X}, size 0x{1:X}" -f $bootText.Start, $bootText.Size)

if(-not $SkipValidate)
{
    & powershell -ExecutionPolicy Bypass -File (Join-Path $repoRoot 'validate-release-package.ps1') -PackageDir $OutputDir
    if($LASTEXITCODE -ne 0)
    {
        throw "Release package validation failed with exit code $LASTEXITCODE."
    }
}
