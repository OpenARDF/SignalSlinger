# SignalSlinger
SignalSlinger is an 80-meter band radio orienteering (ARDF) transmitter kit designed to operate in the Amateur Radio Service 3.5 MHz to 3.7 MHz frequency range. The transmitter supports all common radio orienteering event formats: classic, sprint, and foxoring. Using its own high-accuracy real-time clock, it keeps all transmissions closely synchronized and allows precise start and finish dates and times to be specified.

The SignalSlinger project is 100% Open Source. All software and hardware design documents are available for download from this GitHub repository.

BOM: <a href="https://docs.google.com/spreadsheets/d/182rCsEmR_KNoESYd0NLeVXOi867AKD0zTovbhvcYbqc/edit?usp=sharing">Bill of Material</a>.

User Manual: <a href="https://docs.google.com/document/d/1eX7xH3cDyRNS-MVg13EojpP8IB8bDgrdszM48J0sn7k/edit?usp=sharing">Online Manual</a>.

A matching antenna: <a href="https://github.com/OpenARDF/SignalStreamer">SignalStreamer</a>.

Also, check out SignalSlinger's sibling receiver project: <a href="https://github.com/OpenARDF/SignalSnagger">SignalSnagger</a>.

# SignalSlinger – Programming Instructions

Project: SignalSlinger
Firmware version: v1.1
Supported hardware revisions: 3.5 and 3.4

Firmware files are named to indicate software version and hardware supported:

* SignalSlinger-v1.1-3.5.hex
* SignalSlinger-v1.1-3.4.hex

## IMPORTANT – READ FIRST

You MUST program the firmware file that matches the hardware revision of
your SignalSlinger board.

* Hardware revision 3.5 → SignalSlinger-v1.1-3.5.hex
* Hardware revision 3.4 → SignalSlinger-v1.1-3.4.hex

Programming the wrong file may result in incorrect operation.

## Overview

SignalSlinger firmware is distributed as prebuilt .hex files.
You do NOT need to build the firmware or install an IDE.

Programming is performed using:

* avrdude
* An Atmel-ICE programmer
* The on-board 6-pin programming header labeled P101 (“Programming”)

## Target MCU

Microcontroller: AVR128DA28
Programming interface: UPDI (via Atmel-ICE)

## Hardware Connection

All supported SignalSlinger boards provide a dedicated 6-pin programming
header:

* Reference designator: P101
* Silkscreen label: “Programming”

Connect the Atmel-ICE directly to this header.
No additional wiring, adapters, or external components are required.

## Power

Ensure the SignalSlinger board is powered during programming. The board may be powered by applying 5-12VDC to its primary power input jack, or by connecting an FTDI configuration cable between a USB power source and the 9-pin serial connector.

## Computer Software Requirement

Avrdude must be installed on the host computer.

Typical installation:

* macOS:  brew install avrdude
* Linux: sudo apt install avrdude
* Windows: Use a precompiled avrdude package that supports Atmel-ICE

## Programming Commands

### Hardware revision 3.5

```bash
avrdude -c atmelice_updi -p avr128da28 \
        -U flash:w:SignalSlinger-v1.1-3.5.hex
```

### Hardware revision 3.4

```bash
avrdude -c atmelice_updi -p avr128da28 \
        -U flash:w:SignalSlinger-v1.1-3.4.hex
```

Successful programming will end with:
verification OK

## Notes and Warnings

* Do NOT modify fuses.
* Do NOT disable UPDI.
* No EEPROM programming is required.
* Reprogramming with a newer firmware version uses the same procedure.


