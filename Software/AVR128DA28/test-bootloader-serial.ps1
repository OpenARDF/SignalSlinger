[CmdletBinding()]
param(
    [string]$Port = 'COM6',

    [int]$BootBaud = 115200,

    [int]$AppBaud = 9600,

    [switch]$RequestBootloaderFromApp,

    [switch]$NoReset,

    [switch]$LeaveInBootloader,

    [UInt32]$ScratchAddress = 0x1FE00,

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

function Open-SignalSlingerSerialPort {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PortName,

        [Parameter(Mandatory = $true)]
        [int]$BaudRate
    )

    $serialPort = [System.IO.Ports.SerialPort]::new($PortName, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $serialPort.ReadTimeout = 100
    $serialPort.WriteTimeout = 2000
    $serialPort.DtrEnable = $false
    $serialPort.RtsEnable = $false
    $serialPort.Open()

    return $serialPort
}

function Close-SignalSlingerSerialPort {
    param(
        [System.IO.Ports.SerialPort]$SerialPort
    )

    if($null -ne $SerialPort)
    {
        if($SerialPort.IsOpen)
        {
            $SerialPort.Close()
        }
        $SerialPort.Dispose()
    }
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

function Get-Crc16ForBytes {
    param(
        [Parameter(Mandatory = $true)]
        [byte[]]$Bytes
    )

    [UInt16]$crc = 0xFFFF
    foreach($byte in $Bytes)
    {
        $crc = Update-Crc16 -Crc $crc -Value $byte
    }

    return $crc
}

function Invoke-RawSerial {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory = $true)]
        [byte[]]$Bytes,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedPattern,

        [Parameter(Mandatory = $true)]
        [string]$Description,

        [int]$TimeoutMs = (($FrameByteTimeoutMs * 2) + 750)
    )

    $SerialPort.DiscardInBuffer()
    $SerialPort.Write($Bytes, 0, $Bytes.Length)
    $response = ''
    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
    while([DateTime]::UtcNow -lt $deadline -and $response -notmatch $ExpectedPattern)
    {
        $response += $SerialPort.ReadExisting()
        Start-Sleep -Milliseconds 10
    }

    if($response -notmatch $ExpectedPattern)
    {
        throw "$Description failed. Expected pattern '$ExpectedPattern', received: $response"
    }

    Write-Host ("PASS: {0} -> {1}" -f $Description, $response.Trim())
    return $response
}

function Invoke-BootloaderFrame {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [Parameter(Mandatory = $true)]
        [byte[]]$Frame,

        [Parameter(Mandatory = $true)]
        [string]$ExpectedPattern,

        [Parameter(Mandatory = $true)]
        [string]$Description,

        [int]$TimeoutMs = (($FrameByteTimeoutMs * 2) + 750)
    )

    Invoke-RawSerial -SerialPort $SerialPort -Bytes $Frame -ExpectedPattern $ExpectedPattern -Description $Description -TimeoutMs $TimeoutMs | Out-Null
}

function Request-AppBootloaderReset {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PortName
    )

    $serialPort = Open-SignalSlingerSerialPort -PortName $PortName -BaudRate $AppBaud
    try
    {
        $serialPort.DiscardInBuffer()
        $entryText = Wait-SerialIdle -SerialPort $serialPort -IdleMilliseconds $AppSerialIdleMs -DeadlineMilliseconds $AppSerialIdleDeadlineMs
        $serialPort.Write("`r")
        Start-Sleep -Milliseconds 200
        $entryText += $serialPort.ReadExisting()
        $serialPort.Write("UPD`r")
        Start-Sleep -Milliseconds $AppUpdateCommandSettleMs

        $ackDeadline = [DateTime]::UtcNow.AddMilliseconds($AppUpdateAckDeadlineMs)
        while([DateTime]::UtcNow -lt $ackDeadline -and
              $entryText -notmatch 'Bootloader update' -and
              $entryText -notmatch 'SignalSlinger BL')
        {
            $entryText += $serialPort.ReadExisting()
            Start-Sleep -Milliseconds 25
        }

        if($entryText -notmatch 'Bootloader update' -and $entryText -notmatch 'SignalSlinger BL')
        {
            throw "Application did not acknowledge UPD on $PortName at $AppBaud baud. Received: $entryText"
        }

        return $entryText
    }
    finally
    {
        Close-SignalSlingerSerialPort -SerialPort $serialPort
    }
}

function Invoke-TargetResetAndCatchBootloader {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort
    )

    $stdoutPath = [System.IO.Path]::GetTempFileName()
    $stderrPath = [System.IO.Path]::GetTempFileName()
    $entryText = ''

    try
    {
        $arguments = @(
            '-t', $Tool,
            '-s', $ToolSerial,
            '-i', $Interface,
            '-d', $Device,
            '-cl', $Clock,
            'reset'
        )
        $process = Start-Process -FilePath $AtprogramPath -ArgumentList $arguments -NoNewWindow -PassThru -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
        $afterResetDeadline = $null

        while($entryText -notmatch 'SignalSlinger BL')
        {
            if($process.HasExited)
            {
                if($null -eq $afterResetDeadline)
                {
                    $afterResetDeadline = [DateTime]::UtcNow.AddMilliseconds($BootEntryPulseMs)
                }
                elseif([DateTime]::UtcNow -ge $afterResetDeadline)
                {
                    break
                }
            }

            $SerialPort.Write('U')
            Start-Sleep -Milliseconds 50
            $entryText += $SerialPort.ReadExisting()
        }

        $process.WaitForExit()
        $process.Refresh()
        $stdout = Get-Content -LiteralPath $stdoutPath -Raw
        $stderr = Get-Content -LiteralPath $stderrPath -Raw
        if(-not [string]::IsNullOrWhiteSpace($stdout))
        {
            Write-Host $stdout.TrimEnd()
        }
        if(-not [string]::IsNullOrWhiteSpace($stderr))
        {
            Write-Host $stderr.TrimEnd()
        }
        $exitCode = $process.ExitCode
        if($null -ne $exitCode -and $exitCode -ne 0)
        {
            throw "Target reset failed with exit code $exitCode."
        }
    }
    finally
    {
        Remove-Item -LiteralPath $stdoutPath, $stderrPath -ErrorAction SilentlyContinue
    }

    return $entryText
}

function Enter-Bootloader {
    param(
        [Parameter(Mandatory = $true)]
        [System.IO.Ports.SerialPort]$SerialPort,

        [string]$InitialText = '',

        [switch]$AppResetAlreadyRequested,

        [switch]$AlreadyInBootloader
    )

    $SerialPort.DiscardInBuffer()
    $entryText = $InitialText

    if($AppResetAlreadyRequested)
    {
        $deadline = [DateTime]::UtcNow.AddMilliseconds($BootEntryPulseMs + $BootSettleMs)
        while([DateTime]::UtcNow -lt $deadline -and $entryText -notmatch 'SignalSlinger BL')
        {
            $SerialPort.Write('U')
            Start-Sleep -Milliseconds 50
            $entryText += $SerialPort.ReadExisting()
        }
    }
    elseif(-not $AlreadyInBootloader)
    {
        $entryText = Invoke-TargetResetAndCatchBootloader -SerialPort $SerialPort
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

if($RequestBootloaderFromApp -and $NoReset)
{
    throw '-RequestBootloaderFromApp and -NoReset cannot be used together.'
}
if(($ScratchAddress % $FlashPageBytes) -ne 0 -or $ScratchAddress -lt $ApplicationStart -or $ScratchAddress -gt ($FlashEndExclusive - $FlashPageBytes))
{
    throw ("ScratchAddress must be a writable APPCODE page. Got 0x{0:X}." -f $ScratchAddress)
}
if(-not $RequestBootloaderFromApp -and -not $NoReset)
{
    Assert-PathExists -Path $AtprogramPath -Description 'Atmel Studio atprogram.exe'
}

Write-Host ("Bootloader protocol test: port {0}, app {1} baud, bootloader {2} baud, scratch 0x{3:X}" -f $Port, $AppBaud, $BootBaud, $ScratchAddress)

$entryText = ''
if($RequestBootloaderFromApp)
{
    $entryText = Request-AppBootloaderReset -PortName $Port
}

$serialPort = Open-SignalSlingerSerialPort -PortName $Port -BaudRate $BootBaud
try
{
    $entryText = Enter-Bootloader -SerialPort $serialPort -InitialText $entryText -AppResetAlreadyRequested:$RequestBootloaderFromApp -AlreadyInBootloader:$NoReset
    Write-Host $entryText.Trim()

    Invoke-RawSerial -SerialPort $serialPort -Bytes ([byte[]]@([byte][char]'?')) -ExpectedPattern 'SignalSlinger BL' -Description 'info command' | Out-Null
    Invoke-RawSerial -SerialPort $serialPort -Bytes ([byte[]]@([byte][char]'U')) -ExpectedPattern 'BOOT' -Description 'idempotent update request' | Out-Null
    Invoke-RawSerial -SerialPort $serialPort -Bytes ([byte[]]@([byte][char]'Z')) -ExpectedPattern 'ERR unsupported' -Description 'unsupported command' | Out-Null

    $eraseScratch = New-BootloaderFrame -Command ([byte][char]'E') -Address $ScratchAddress
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $eraseScratch -ExpectedPattern 'OK erase' -Description 'erase scratch page'

    $pattern = [byte[]]::new($FlashPageBytes)
    for($i = 0; $i -lt $pattern.Length; $i++)
    {
        $pattern[$i] = [byte](($i * 37 + 11) -band 0xFF)
    }
    $writeScratch = New-BootloaderFrame -Command ([byte][char]'W') -Address $ScratchAddress -Payload $pattern
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $writeScratch -ExpectedPattern 'OK write' -Description 'write scratch page'

    $expectedScratchCrc = Get-Crc16ForBytes -Bytes $pattern
    $crcScratch = New-BootloaderFrame -Command ([byte][char]'C') -Address $ScratchAddress
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $crcScratch -ExpectedPattern ("OK crc 0x{0:X8} {1:X4}" -f $ScratchAddress, $expectedScratchCrc) -Description 'verify scratch page CRC'

    $badCrc = New-BootloaderFrame -Command ([byte][char]'E') -Address $ScratchAddress
    $badCrc[$badCrc.Length - 1] = $badCrc[$badCrc.Length - 1] -bxor 0x01
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $badCrc -ExpectedPattern 'ERR crc' -Description 'reject bad CRC'

    $unaligned = New-BootloaderFrame -Command ([byte][char]'E') -Address ($ScratchAddress + 1)
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $unaligned -ExpectedPattern 'ERR address' -Description 'reject unaligned address'

    $bootAddress = New-BootloaderFrame -Command ([byte][char]'E') -Address 0x0000
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $bootAddress -ExpectedPattern 'ERR address' -Description 'reject boot-section address'

    $bootCrcAddress = New-BootloaderFrame -Command ([byte][char]'C') -Address 0x0000
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $bootCrcAddress -ExpectedPattern 'ERR address' -Description 'reject boot-section CRC read'

    $pastFlash = New-BootloaderFrame -Command ([byte][char]'E') -Address $FlashEndExclusive
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $pastFlash -ExpectedPattern 'ERR address' -Description 'reject past-flash address'

    $truncatedWrite = [byte[]]@(
        [byte][char]'W',
        [byte]($ScratchAddress -band 0xFF),
        [byte](($ScratchAddress -shr 8) -band 0xFF),
        [byte](($ScratchAddress -shr 16) -band 0xFF),
        [byte](($ScratchAddress -shr 24) -band 0xFF)
    )
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $truncatedWrite -ExpectedPattern 'ERR timeout' -Description 'reject truncated write frame' -TimeoutMs 3500

    Invoke-RawSerial -SerialPort $serialPort -Bytes ([byte[]]@([byte][char]'?')) -ExpectedPattern 'SignalSlinger BL' -Description 'respond after malformed frames' | Out-Null
    Invoke-BootloaderFrame -SerialPort $serialPort -Frame $eraseScratch -ExpectedPattern 'OK erase' -Description 'restore scratch page to erased state'

    if(-not $LeaveInBootloader)
    {
        $serialPort.Write('R')
        Write-Host 'PASS: run app command sent'
    }
}
finally
{
    Close-SignalSlingerSerialPort -SerialPort $serialPort
}

Write-Host 'Bootloader protocol test passed.'
