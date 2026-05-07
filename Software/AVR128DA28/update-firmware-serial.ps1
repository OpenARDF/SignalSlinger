[CmdletBinding()]
param(
    [string]$Port = 'COM6',

    [int]$Baud = 9600,

    [string]$HexPath = '',

    [switch]$SkipBuild,

    [switch]$DryRun,

    [switch]$NoReset,

    [switch]$RequestBootloaderFromApp,

    [switch]$VerifyWithUpdi,

    [string]$Tool = 'atmelice',

    [string]$ToolSerial = 'J41800053674',

    [string]$Interface = 'UPDI',

    [string]$Device = 'avr128da28',

    [string]$Clock = '100kHz'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$ApplicationStart = 0x4000
$FlashEndExclusive = 0x20000
$FlashPageBytes = 512
$FrameByteTimeoutMs = 1000
$BootEntryPulseMs = 1800
$BootSettleMs = 2500
$AppUpdateCommandSettleMs = 250
$AppSerialIdleMs = 750
$AppSerialIdleDeadlineMs = 6000
$AppUpdateAckDeadlineMs = 3000
$AtprogramPath = 'C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe'

if([string]::IsNullOrWhiteSpace($HexPath))
{
    $HexPath = Join-Path $PSScriptRoot 'SignalSlinger\Release\SignalSlinger.hex'
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
            throw "Invalid Intel HEX line without ':' prefix: $line"
        }

        $byteCount = ConvertFrom-HexByte $line.Substring(1, 2)
        $recordAddress = [Convert]::ToUInt16($line.Substring(3, 4), 16)
        $recordType = ConvertFrom-HexByte $line.Substring(7, 2)
        $expectedLength = 11 + ($byteCount * 2)
        if($line.Length -ne $expectedLength)
        {
            throw "Invalid Intel HEX line length: $line"
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
            throw "Intel HEX checksum failed: $line"
        }

        switch($recordType)
        {
            0 {
                $absoluteAddress = $upperAddress + $recordAddress
                for($i = 0; $i -lt $byteCount; $i++)
                {
                    $address = $absoluteAddress + $i
                    if($address -lt $ApplicationStart -or $address -ge $FlashEndExclusive)
                    {
                        throw ("HEX data address 0x{0:X} is outside APPCODE 0x{1:X}..0x{2:X}." -f $address, $ApplicationStart, ($FlashEndExclusive - 1))
                    }

                    $bytesByAddress[$address] = $data[$i]
                }
            }
            1 {
                return $bytesByAddress
            }
            2 {
                if($byteCount -ne 2)
                {
                    throw "Invalid extended segment address record length."
                }
                $upperAddress = ((([int]$data[0] -shl 8) -bor [int]$data[1]) -shl 4)
            }
            4 {
                if($byteCount -ne 2)
                {
                    throw "Invalid extended linear address record length."
                }
                $upperAddress = ((([int]$data[0] -shl 8) -bor [int]$data[1]) -shl 16)
            }
            3 {}
            5 {}
            default {
                throw "Unsupported Intel HEX record type $recordType."
            }
        }
    }

    return $bytesByAddress
}

function Get-PagesFromBytes {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$BytesByAddress
    )

    $pages = @{}

    foreach($address in $BytesByAddress.Keys)
    {
        $pageAddress = [int]$address - ([int]$address % $FlashPageBytes)
        if(-not $pages.ContainsKey($pageAddress))
        {
            $page = [byte[]]::new($FlashPageBytes)
            for($i = 0; $i -lt $FlashPageBytes; $i++)
            {
                $page[$i] = 0xFF
            }
            $pages[$pageAddress] = $page
        }

        $pageOffset = [int]$address - $pageAddress
        $pages[$pageAddress][$pageOffset] = [byte]$BytesByAddress[$address]
    }

    return $pages
}

function Update-Crc16 {
    param(
        [UInt16]$Crc,
        [byte]$Value
    )

    $crcValue = [int]($Crc -bxor ([UInt16]$Value -shl 8))
    for($bit = 0; $bit -lt 8; $bit++)
    {
        if(($crcValue -band 0x8000) -ne 0)
        {
            $crcValue = (($crcValue -shl 1) -bxor 0x1021) -band 0xFFFF
        }
        else
        {
            $crcValue = ($crcValue -shl 1) -band 0xFFFF
        }
    }

    return [UInt16]$crcValue
}

function New-BootloaderFrame {
    param(
        [Parameter(Mandatory = $true)]
        [byte]$Command,

        [Parameter(Mandatory = $true)]
        [UInt32]$Address,

        [byte[]]$Payload
    )

    $body = [System.Collections.Generic.List[byte]]::new()
    $body.Add($Command)
    $body.Add([byte]($Address -band 0xFF))
    $body.Add([byte](($Address -shr 8) -band 0xFF))
    $body.Add([byte](($Address -shr 16) -band 0xFF))
    $body.Add([byte](($Address -shr 24) -band 0xFF))

    if($Payload)
    {
        $body.AddRange($Payload)
    }

    [UInt16]$crc = 0xFFFF
    foreach($byte in $body)
    {
        $crc = Update-Crc16 -Crc $crc -Value $byte
    }

    $body.Add([byte]($crc -band 0xFF))
    $body.Add([byte](($crc -shr 8) -band 0xFF))

    return $body.ToArray()
}

function Read-SerialText {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory = $true)]
        [int]$Milliseconds
    )

    $text = ''
    $deadline = [DateTime]::UtcNow.AddMilliseconds($Milliseconds)
    while([DateTime]::UtcNow -lt $deadline)
    {
        $text += $SerialPort.ReadExisting()
        Start-Sleep -Milliseconds 25
    }

    return $text
}

function Wait-SerialIdle {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory = $true)]
        [int]$IdleMilliseconds,

        [Parameter(Mandatory = $true)]
        [int]$DeadlineMilliseconds
    )

    $text = ''
    $lastByteTime = [DateTime]::UtcNow
    $deadline = [DateTime]::UtcNow.AddMilliseconds($DeadlineMilliseconds)

    while([DateTime]::UtcNow -lt $deadline)
    {
        $chunk = $SerialPort.ReadExisting()
        if($chunk.Length -gt 0)
        {
            $text += $chunk
            $lastByteTime = [DateTime]::UtcNow
        }
        elseif(([DateTime]::UtcNow - $lastByteTime).TotalMilliseconds -ge $IdleMilliseconds)
        {
            break
        }

        Start-Sleep -Milliseconds 25
    }

    return $text
}

function Enter-Bootloader {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [switch]$RequestFromApp,

        [switch]$AlreadyInBootloader
    )

    $SerialPort.DiscardInBuffer()

    if($RequestFromApp)
    {
        $entryText = Wait-SerialIdle -SerialPort $SerialPort -IdleMilliseconds $AppSerialIdleMs -DeadlineMilliseconds $AppSerialIdleDeadlineMs
        $SerialPort.Write("`r")
        Start-Sleep -Milliseconds 200
        $entryText += $SerialPort.ReadExisting()

        $SerialPort.Write("UPD`r")
        Start-Sleep -Milliseconds $AppUpdateCommandSettleMs

        $ackDeadline = [DateTime]::UtcNow.AddMilliseconds($AppUpdateAckDeadlineMs)
        while([DateTime]::UtcNow -lt $ackDeadline -and
              $entryText -notmatch 'Bootloader update' -and
              $entryText -notmatch 'SignalSlinger BL')
        {
            $entryText += $SerialPort.ReadExisting()
            Start-Sleep -Milliseconds 25
        }

        if($entryText -notmatch 'SignalSlinger BL')
        {
            $deadline = [DateTime]::UtcNow.AddMilliseconds($BootEntryPulseMs + $BootSettleMs)
            while([DateTime]::UtcNow -lt $deadline -and $entryText -notmatch 'SignalSlinger BL')
            {
                $SerialPort.Write('U')
                Start-Sleep -Milliseconds 50
                $entryText += $SerialPort.ReadExisting()
            }
        }
    }
    elseif(-not $AlreadyInBootloader)
    {
        & $AtprogramPath `
            -t $Tool `
            -s $ToolSerial `
            -i $Interface `
            -d $Device `
            -cl $Clock `
            reset | Out-Host

        if($LASTEXITCODE -ne 0)
        {
            throw "Target reset failed with exit code $LASTEXITCODE."
        }

        $deadline = [DateTime]::UtcNow.AddMilliseconds($BootEntryPulseMs)
        $entryText = ''
        while([DateTime]::UtcNow -lt $deadline -and $entryText -notmatch 'SignalSlinger BL')
        {
            $SerialPort.Write('U')
            Start-Sleep -Milliseconds 50
            $entryText += $SerialPort.ReadExisting()
        }
    }
    else
    {
        $entryText = ''
    }

    $entryText += Read-SerialText -SerialPort $SerialPort -Milliseconds $BootSettleMs
    if($entryText -notmatch 'BOOT' -and $entryText -notmatch 'Waiting for updater')
    {
        $SerialPort.Write('?')
        $entryText += Read-SerialText -SerialPort $SerialPort -Milliseconds $BootSettleMs
    }

    if($entryText -notmatch 'SignalSlinger BL')
    {
        throw "Bootloader did not respond on $Port. Received: $entryText"
    }

    $SerialPort.DiscardInBuffer()
    return $entryText
}

function Invoke-BootloaderFrame {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory = $true)]
        [byte[]]$Frame,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedResponse,

        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    $SerialPort.DiscardInBuffer()
    $SerialPort.Write($Frame, 0, $Frame.Length)
    $response = ''
    $deadline = [DateTime]::UtcNow.AddMilliseconds(($FrameByteTimeoutMs * 2) + 750)
    while([DateTime]::UtcNow -lt $deadline -and $response -notmatch [regex]::Escape($ExpectedResponse))
    {
        $response += $SerialPort.ReadExisting()
        Start-Sleep -Milliseconds 10
    }

    if($response -notmatch [regex]::Escape($ExpectedResponse))
    {
        throw "$Description failed. Expected '$ExpectedResponse', received: $response"
    }

    return $response
}

if($RequestBootloaderFromApp -and $NoReset)
{
    throw '-RequestBootloaderFromApp and -NoReset cannot be used together.'
}

if(((-not $RequestBootloaderFromApp) -and (-not $NoReset)) -or $VerifyWithUpdi)
{
    Assert-PathExists -Path $AtprogramPath -Description 'Atmel Studio atprogram.exe'
}

if(-not $SkipBuild)
{
    & (Join-Path $PSScriptRoot 'build-relocated-firmware.ps1') -Configuration Release
    if($LASTEXITCODE -ne 0)
    {
        throw "Relocated firmware build failed with exit code $LASTEXITCODE."
    }
}

Assert-PathExists -Path $HexPath -Description 'Relocated firmware HEX'

$bytesByAddress = Get-IntelHexBytes -Path $HexPath
if($bytesByAddress.Count -eq 0)
{
    throw "No data records found in $HexPath."
}

$pages = Get-PagesFromBytes -BytesByAddress $bytesByAddress
$pageAddresses = @($pages.Keys | Sort-Object { [int]$_ })
$resetVectorPageAddress = $ApplicationStart
if(-not $pages.ContainsKey($resetVectorPageAddress))
{
    throw ("Relocated HEX does not contain the reset-vector page at 0x{0:X}." -f $resetVectorPageAddress)
}

$orderedWrites = @($pageAddresses | Where-Object { [int]$_ -ne $resetVectorPageAddress })
$orderedWrites += $resetVectorPageAddress

$firstAddress = [int]($bytesByAddress.Keys | Sort-Object { [int]$_ } | Select-Object -First 1)
$lastAddress = [int]($bytesByAddress.Keys | Sort-Object { [int]$_ } | Select-Object -Last 1)
Write-Host ("Relocated HEX: {0} bytes across {1} pages, 0x{2:X}..0x{3:X}" -f $bytesByAddress.Count, $pageAddresses.Count, $firstAddress, $lastAddress)
Write-Host ("Reset-vector page 0x{0:X} will be erased first and written last." -f $resetVectorPageAddress)

if($DryRun)
{
    Write-Host 'Dry run only; no serial frames sent.'
    exit 0
}

$serialPort = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 100
$serialPort.WriteTimeout = 2000
$serialPort.DtrEnable = $false
$serialPort.RtsEnable = $false
$serialPort.Open()

try
{
    $entryText = Enter-Bootloader -SerialPort $serialPort -RequestFromApp:$RequestBootloaderFromApp -AlreadyInBootloader:$NoReset
    Write-Host $entryText.Trim()

    $eraseResetFrame = New-BootloaderFrame -Command ([byte][char]'E') -Address $resetVectorPageAddress
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $eraseResetFrame -ExpectedResponse 'OK erase' -Description ("Erase reset-vector page 0x{0:X}" -f $resetVectorPageAddress) | Out-Null
    Write-Host ("Erased reset-vector page 0x{0:X}." -f $resetVectorPageAddress)

    $totalPages = $orderedWrites.Count
    $pageNumber = 0
    foreach($pageAddress in $orderedWrites)
    {
        $pageNumber++
        if([int]$pageAddress -ne $resetVectorPageAddress)
        {
            $eraseFrame = New-BootloaderFrame -Command ([byte][char]'E') -Address ([UInt32]$pageAddress)
            Invoke-BootloaderFrame -SerialPort $serialPort -Frame $eraseFrame -ExpectedResponse 'OK erase' -Description ("Erase page 0x{0:X}" -f $pageAddress) | Out-Null
        }

        $writeFrame = New-BootloaderFrame -Command ([byte][char]'W') -Address ([UInt32]$pageAddress) -Payload $pages[$pageAddress]
        Invoke-BootloaderFrame -SerialPort $serialPort -Frame $writeFrame -ExpectedResponse 'OK write' -Description ("Write page 0x{0:X}" -f $pageAddress) | Out-Null

        if(($pageNumber % 16) -eq 0 -or $pageNumber -eq $totalPages)
        {
            Write-Host ("Wrote page {0}/{1}: 0x{2:X}" -f $pageNumber, $totalPages, $pageAddress)
        }
        Write-Progress -Activity 'Updating SignalSlinger firmware over bootloader' -Status ("Page {0}/{1}: 0x{2:X}" -f $pageNumber, $totalPages, $pageAddress) -PercentComplete (($pageNumber / $totalPages) * 100)
    }

    Write-Progress -Activity 'Updating SignalSlinger firmware over bootloader' -Completed
    Write-Host ("Serial update complete: wrote {0} pages from {1}" -f $totalPages, $HexPath)

    $serialPort.Write('R')
    Write-Host 'Sent run-app command.'
}
finally
{
    if($serialPort.IsOpen)
    {
        $serialPort.Close()
    }
    $serialPort.Dispose()
}

if($VerifyWithUpdi)
{
    & $AtprogramPath `
        -t $Tool `
        -s $ToolSerial `
        -i $Interface `
        -d $Device `
        -cl $Clock `
        verify `
        -fl `
        -f $HexPath

    if($LASTEXITCODE -ne 0)
    {
        throw "UPDI verification failed with exit code $LASTEXITCODE."
    }
}
