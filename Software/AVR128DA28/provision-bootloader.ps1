[CmdletBinding()]
param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',

    [ValidateSet('Auto', 'Atprogram', 'Pymcuprog')]
    [string]$Backend = 'Auto',

    [string]$Port = 'COM6',

    [string]$BootloaderHexPath = '',

    [string]$ApplicationHexPath = '',

    [string]$CombinedHexPath = '',

    [switch]$SkipBuild,

    [switch]$BuildOnly,

    [switch]$ProgramFuses,

    [switch]$ConfirmFuseWrite,

    [switch]$SkipSerialValidation,

    [switch]$DryRun,

    [switch]$CheckPrereqs,

    [string]$AtprogramPath = 'C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe',

    [string]$PymcuprogCommand = 'pymcuprog',

    [string]$Tool = 'atmelice',

    [string]$ToolSerial = 'J41800053674',

    [string]$Interface = 'UPDI',

    [string]$Device = 'avr128da28',

    [string]$Clock = '100kHz'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$BootSizeFuseOffset = 8
$CodeSizeFuseOffset = 7
$DesiredBootSize = 0x10
$DesiredCodeSize = 0x00
$FuseReadBytes = 16

if([string]::IsNullOrWhiteSpace($BootloaderHexPath))
{
    $BootloaderHexPath = Join-Path $PSScriptRoot 'bootloader\Release\SignalSlingerBootloader.hex'
}
if([string]::IsNullOrWhiteSpace($ApplicationHexPath))
{
    $ApplicationHexPath = Join-Path $PSScriptRoot 'SignalSlinger\Release\SignalSlinger.hex'
}
if([string]::IsNullOrWhiteSpace($CombinedHexPath))
{
    $CombinedHexPath = Join-Path $PSScriptRoot 'tmp\SignalSlinger-bootloader-combined.hex'
}

if($CheckPrereqs -and ($ProgramFuses -or $ConfirmFuseWrite))
{
    throw '-CheckPrereqs cannot be combined with fuse-write switches.'
}

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

function Test-CommandAvailable {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Command
    )

    return $null -ne (Get-Command $Command -ErrorAction SilentlyContinue)
}

function Add-PrereqResult {
    param(
        [System.Collections.Generic.List[object]]$Results,

        [Parameter(Mandatory = $true)]
        [string]$Name,

        [Parameter(Mandatory = $true)]
        [bool]$Present,

        [Parameter(Mandatory = $true)]
        [bool]$Required,

        [Parameter(Mandatory = $true)]
        [string]$Install
    )

    $Results.Add([pscustomobject]@{
        Name = $Name
        Present = $Present
        Required = $Required
        Install = $Install
    })
}

function Test-Prerequisites {
    $results = [System.Collections.Generic.List[object]]::new()
    $requiresBuildTools = -not $SkipBuild
    $requiresAtprogram = $Backend -eq 'Atprogram' -or ($Backend -eq 'Auto' -and (Test-Path -LiteralPath $AtprogramPath))
    $requiresPymcuprog = $Backend -eq 'Pymcuprog' -or ($Backend -eq 'Auto' -and -not (Test-Path -LiteralPath $AtprogramPath))

    Add-PrereqResult -Results $results -Name 'PowerShell' -Present ($PSVersionTable.PSVersion.Major -ge 5) -Required $true -Install 'Windows: built in or install PowerShell 7. macOS: brew install --cask powershell'
    Add-PrereqResult -Results $results -Name 'atprogram' -Present (Test-Path -LiteralPath $AtprogramPath) -Required $requiresAtprogram -Install 'Install Microchip Studio 7 on Windows, or pass -AtprogramPath to atprogram.exe.'
    Add-PrereqResult -Results $results -Name 'pymcuprog' -Present (Test-CommandAvailable $PymcuprogCommand) -Required $requiresPymcuprog -Install 'Install Python, then run: python -m pip install pymcuprog'
    Add-PrereqResult -Results $results -Name 'Python' -Present ((Test-CommandAvailable 'python') -or (Test-CommandAvailable 'python3')) -Required $requiresPymcuprog -Install 'Windows: install from python.org. macOS: use system Python, python.org, or brew install python.'
    Add-PrereqResult -Results $results -Name 'Bootloader build script' -Present (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'build-bootloader.ps1')) -Required $requiresBuildTools -Install 'Use the SignalSlinger repo with Software/AVR128DA28 build scripts.'
    Add-PrereqResult -Results $results -Name 'Relocated app build script' -Present (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'build-relocated-firmware.ps1')) -Required $requiresBuildTools -Install 'Use the SignalSlinger repo with Software/AVR128DA28 build scripts.'
    Add-PrereqResult -Results $results -Name 'Bootloader HEX' -Present (Test-Path -LiteralPath $BootloaderHexPath) -Required $SkipBuild -Install 'Build on Windows or provide -BootloaderHexPath to a release HEX.'
    Add-PrereqResult -Results $results -Name 'Relocated app HEX' -Present (Test-Path -LiteralPath $ApplicationHexPath) -Required $SkipBuild -Install 'Build on Windows or provide -ApplicationHexPath to a release HEX.'
    Add-PrereqResult -Results $results -Name 'Serial validation script' -Present (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'test-bootloader-serial.ps1')) -Required (-not $SkipSerialValidation) -Install 'Use the SignalSlinger repo with test-bootloader-serial.ps1, or pass -SkipSerialValidation.'

    Write-Host 'Prerequisite check only; not touching target hardware.'
    Write-Host ("Requested backend: {0}" -f $Backend)
    Write-Host ("Build required: {0}" -f $requiresBuildTools)
    Write-Host ("Serial validation required: {0}" -f (-not $SkipSerialValidation))
    Write-Host ''
    $results | Format-Table -AutoSize | Out-String | Write-Host

    $missingRequired = @($results | Where-Object { $_.Required -and -not $_.Present })
    if($missingRequired.Count -gt 0)
    {
        Write-Host 'Missing required prerequisites:'
        foreach($item in $missingRequired)
        {
            Write-Host ("- {0}: {1}" -f $item.Name, $item.Install)
        }
        exit 1
    }

    Write-Host 'All required prerequisites are present for the selected mode.'
    exit 0
}

function Invoke-CheckedCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,

        [Parameter(Mandatory = $true)]
        [string[]]$Arguments,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    Write-Host ('> {0} {1}' -f $FilePath, ($Arguments -join ' '))
    if($DryRun)
    {
        return
    }

    & $FilePath @Arguments | Out-Host
    if($LASTEXITCODE -ne 0)
    {
        throw "$Description failed with exit code $LASTEXITCODE."
    }
}

function Resolve-Backend {
    if($Backend -ne 'Auto')
    {
        return $Backend
    }
    if(Test-Path -LiteralPath $AtprogramPath)
    {
        return 'Atprogram'
    }
    $command = Get-Command $PymcuprogCommand -ErrorAction SilentlyContinue
    if($null -ne $command)
    {
        return 'Pymcuprog'
    }

    throw "No programming backend found. Install Microchip Studio/atprogram on Windows, or install pymcuprog for macOS/Windows."
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
            throw "Invalid Intel HEX line without ':' prefix in $Path`: $line"
        }

        $byteCount = ConvertFrom-HexByte $line.Substring(1, 2)
        $recordAddress = [Convert]::ToUInt16($line.Substring(3, 4), 16)
        $recordType = ConvertFrom-HexByte $line.Substring(7, 2)
        $expectedLength = 11 + ($byteCount * 2)
        if($line.Length -ne $expectedLength)
        {
            throw "Invalid Intel HEX line length in $Path`: $line"
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
            throw "Intel HEX checksum failed in $Path`: $line"
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
                throw "Unsupported Intel HEX record type $recordType in $Path."
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

function Merge-IntelHexFiles {
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
            throw ("HEX overlap at 0x{0:X} between bootloader and application." -f $intAddress)
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

function Get-AtprogramBaseArgs {
    return @('-t', $Tool, '-s', $ToolSerial, '-i', $Interface, '-d', $Device, '-cl', $Clock)
}

function Get-PymcuprogBaseArgs {
    return @('-t', $Tool, '-s', $ToolSerial, '-i', $Interface.ToLowerInvariant(), '-d', $Device, '-c', $Clock)
}

function Read-Fuses {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ResolvedBackend
    )

    $tempPath = [System.IO.Path]::GetTempFileName()
    try
    {
        if($ResolvedBackend -eq 'Atprogram')
        {
            $args = (Get-AtprogramBaseArgs) + @('read', '-fs', '--format', 'bin', '-s', "$FuseReadBytes", '-f', $tempPath)
            Invoke-CheckedCommand -FilePath $AtprogramPath -Arguments $args -Description 'Read fuses'
        }
        else
        {
            $args = @('read') + (Get-PymcuprogBaseArgs) + @('-m', 'fuses', '-b', "$FuseReadBytes", '-f', $tempPath)
            Invoke-CheckedCommand -FilePath $PymcuprogCommand -Arguments $args -Description 'Read fuses'
        }

        if($DryRun)
        {
            return ,([byte[]]::new(0))
        }
        return ,([System.IO.File]::ReadAllBytes($tempPath))
    }
    finally
    {
        Remove-Item -LiteralPath $tempPath -ErrorAction SilentlyContinue
    }
}

function Write-FuseByte {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ResolvedBackend,

        [Parameter(Mandatory = $true)]
        [int]$Offset,

        [Parameter(Mandatory = $true)]
        [byte]$Value
    )

    $hexValue = '{0:X2}' -f $Value
    if($ResolvedBackend -eq 'Atprogram')
    {
        $args = (Get-AtprogramBaseArgs) + @('write', '-fs', '-o', "$Offset", '--values', $hexValue, '-v')
        Invoke-CheckedCommand -FilePath $AtprogramPath -Arguments $args -Description ("Write fuse offset {0}" -f $Offset)
    }
    else
    {
        $args = @('write') + (Get-PymcuprogBaseArgs) + @('-m', 'fuses', '-o', "$Offset", '-l', ('0x{0}' -f $hexValue))
        Invoke-CheckedCommand -FilePath $PymcuprogCommand -Arguments $args -Description ("Write fuse offset {0}" -f $Offset)
    }
}

function Show-FuseSummary {
    param(
        [byte[]]$Fuses
    )

    if($Fuses.Length -le $BootSizeFuseOffset)
    {
        Write-Host 'Fuse summary unavailable in dry run.'
        return
    }

    Write-Host ("Fuse CODESIZE offset {0}: 0x{1:X2}" -f $CodeSizeFuseOffset, $Fuses[$CodeSizeFuseOffset])
    Write-Host ("Fuse BOOTSIZE offset {0}: 0x{1:X2}" -f $BootSizeFuseOffset, $Fuses[$BootSizeFuseOffset])
}

if($CheckPrereqs)
{
    Test-Prerequisites
}

$resolvedBackend = Resolve-Backend
Write-Host "Provisioning backend: $resolvedBackend"

if($resolvedBackend -eq 'Atprogram')
{
    Assert-PathExists -Path $AtprogramPath -Description 'Atmel Studio atprogram.exe'
}
elseif($null -eq (Get-Command $PymcuprogCommand -ErrorAction SilentlyContinue))
{
    throw "pymcuprog command not found: $PymcuprogCommand"
}

if($ProgramFuses -and -not $ConfirmFuseWrite)
{
    throw 'Fuse programming requires both -ProgramFuses and -ConfirmFuseWrite.'
}

if(-not $SkipBuild)
{
    & (Join-Path $PSScriptRoot 'build-bootloader.ps1') -Configuration $Configuration
    if($LASTEXITCODE -ne 0)
    {
        throw "Bootloader build failed with exit code $LASTEXITCODE."
    }

    & (Join-Path $PSScriptRoot 'build-relocated-firmware.ps1') -Configuration $Configuration
    if($LASTEXITCODE -ne 0)
    {
        throw "Relocated application build failed with exit code $LASTEXITCODE."
    }
}

Assert-PathExists -Path $BootloaderHexPath -Description 'Bootloader HEX'
Assert-PathExists -Path $ApplicationHexPath -Description 'Relocated application HEX'

$mergeSummary = Merge-IntelHexFiles -BootloaderPath $BootloaderHexPath -ApplicationPath $ApplicationHexPath -OutputPath $CombinedHexPath
Write-Host ("Combined HEX: {0} bytes ({1} bootloader + {2} app) -> {3}" -f $mergeSummary.CombinedBytes, $mergeSummary.BootloaderBytes, $mergeSummary.ApplicationBytes, $CombinedHexPath)

if($BuildOnly)
{
    Write-Host 'BuildOnly requested; not touching target.'
    exit 0
}

$initialFuses = Read-Fuses -ResolvedBackend $resolvedBackend
Show-FuseSummary -Fuses $initialFuses

$needsCodeSize = $initialFuses.Length -gt $CodeSizeFuseOffset -and $initialFuses[$CodeSizeFuseOffset] -ne $DesiredCodeSize
$needsBootSize = $initialFuses.Length -gt $BootSizeFuseOffset -and $initialFuses[$BootSizeFuseOffset] -ne $DesiredBootSize

if(($needsCodeSize -or $needsBootSize) -and -not $ProgramFuses)
{
    Write-Warning ("Fuse values do not match bootloader requirements. Re-run with -ProgramFuses -ConfirmFuseWrite to set CODESIZE=0x{0:X2}, BOOTSIZE=0x{1:X2}." -f $DesiredCodeSize, $DesiredBootSize)
}

if($resolvedBackend -eq 'Atprogram')
{
    Invoke-CheckedCommand -FilePath $AtprogramPath -Arguments ((Get-AtprogramBaseArgs) + @('chiperase')) -Description 'Chip erase'
    Invoke-CheckedCommand -FilePath $AtprogramPath -Arguments ((Get-AtprogramBaseArgs) + @('program', '-fl', '--verify', '-f', $CombinedHexPath)) -Description 'Program combined flash'
}
else
{
    Invoke-CheckedCommand -FilePath $PymcuprogCommand -Arguments (@('write') + (Get-PymcuprogBaseArgs) + @('-f', $CombinedHexPath, '--erase', '--verify')) -Description 'Program combined flash'
}

if($ProgramFuses)
{
    Write-Host ("Programming required fuses: CODESIZE=0x{0:X2}, BOOTSIZE=0x{1:X2}" -f $DesiredCodeSize, $DesiredBootSize)
    Write-FuseByte -ResolvedBackend $resolvedBackend -Offset $CodeSizeFuseOffset -Value ([byte]$DesiredCodeSize)
    Write-FuseByte -ResolvedBackend $resolvedBackend -Offset $BootSizeFuseOffset -Value ([byte]$DesiredBootSize)
}

$finalFuses = Read-Fuses -ResolvedBackend $resolvedBackend
Show-FuseSummary -Fuses $finalFuses
if(-not $DryRun -and ($finalFuses[$CodeSizeFuseOffset] -ne $DesiredCodeSize -or $finalFuses[$BootSizeFuseOffset] -ne $DesiredBootSize))
{
    throw ("Fuse verification failed. Expected CODESIZE=0x{0:X2}, BOOTSIZE=0x{1:X2}." -f $DesiredCodeSize, $DesiredBootSize)
}

if(-not $SkipSerialValidation)
{
    Start-Sleep -Seconds 3
    $testScript = Join-Path $PSScriptRoot 'test-bootloader-serial.ps1'
    $validationPassed = $false
    for($attempt = 1; $attempt -le 2 -and -not $validationPassed; $attempt++)
    {
        Write-Host ("Running bootloader serial validation attempt {0}/2..." -f $attempt)
        & powershell -ExecutionPolicy Bypass -File $testScript -Port $Port
        if($LASTEXITCODE -eq 0)
        {
            $validationPassed = $true
        }
        elseif($attempt -lt 2)
        {
            Write-Warning "Bootloader serial validation failed with exit code $LASTEXITCODE; retrying after serial settle."
            Start-Sleep -Seconds 3
        }
    }

    if(-not $validationPassed)
    {
        throw "Bootloader serial validation failed."
    }
}

Write-Host 'Bootloader provisioning completed successfully.'
