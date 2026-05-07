# SignalSlinger Bootloader

This directory contains the first-stage SignalSlinger AVR128DA28 bootloader work.

Current scope:

- reserve a 16 KiB boot section
- keep the application start fixed at byte address `0x4000`
- use `USART1` on the existing SignalSlinger serial pins, `PC0` TX and `PC1` RX
- stay in the bootloader when the front-panel switch is held at reset, when no valid app reset vector is present, or when the serial updater sends `U` during the startup window
- otherwise jump to the relocated application

The bootloader now accepts a deliberately small page programming protocol. It only erases or writes full, page-aligned APPCODE pages and rejects any frame whose address would touch the boot section.

## Address Map

The AVR128DA28 has 131072 bytes of program flash and 512-byte flash pages. `BOOTSIZE` and `CODESIZE` fuses are expressed in 512-byte pages.

Initial bootloader allocation:

- `BOOTSIZE = 32` pages
- boot section: `0x00000` through `0x03FFF`
- relocated application start: `0x04000`
- `CODESIZE = 0`, leaving the remainder as application code space

## Build

From `Software/AVR128DA28` on the Windows/Microchip Studio environment:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-bootloader.ps1 -Configuration Release
```

Build the matching relocated application with:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-relocated-firmware.ps1 -Configuration Release
```

That helper uses a temporary makefile and injects `-Wl,--section-start=.text=0x4000`; it does not alter the normal `SignalSlinger/Release/Makefile`.

## Serial Update Frames

All multi-byte fields are little-endian. CRC is CRC-16/CCITT-FALSE initialized to `0xFFFF` and covers the command byte plus the frame body, but not the transmitted CRC bytes.

- `?`: print bootloader information
- `R`: jump to the app when the app reset vector is programmed
- `E <addr:u32> <crc:u16>`: erase one 512-byte page
- `W <addr:u32> <payload:512 bytes> <crc:u16>`: write one erased 512-byte page

`addr` must be page-aligned and the complete page must be inside `0x04000` through `0x1FFFF`. The bootloader responds with `OK erase`, `OK write`, or `ERR ...`.

## Serial Firmware Update

Build the relocated application, then update a unit that is already running the bootloader:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-relocated-firmware.ps1 -Configuration Release
powershell -ExecutionPolicy Bypass -File .\update-firmware-serial.ps1 -Port COM6 -SkipBuild -NoReset
```

For a normal field update from the running application, let the script send `UPD` to the app and catch the bootloader entry window:

```powershell
powershell -ExecutionPolicy Bypass -File .\update-firmware-serial.ps1 -Port COM6 -RequestBootloaderFromApp
```

For bench validation with Atmel-ICE attached, add `-VerifyWithUpdi` to verify the programmed app flash after the serial update:

```powershell
powershell -ExecutionPolicy Bypass -File .\update-firmware-serial.ps1 -Port COM6 -RequestBootloaderFromApp -VerifyWithUpdi
```

The updater parses the relocated Intel HEX file, rejects records outside APPCODE, erases the reset-vector page first, writes all other pages, then writes the reset-vector page last.

## Provisioning Notes

Do not program bootloader fuses casually. The safe provisioning flow should remain:

1. build the bootloader
2. program the bootloader over UPDI
3. set `BOOTSIZE = 32` and `CODESIZE = 0`
4. build and program a relocated application
5. verify serial entry, timeout jump, and switch-held entry

UPDI must remain available as the recovery path throughout bootloader development.
