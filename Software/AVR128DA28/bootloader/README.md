# SignalSlinger Bootloader

This directory contains the first-stage SignalSlinger AVR128DA28 bootloader work.

Current scope:

- reserve a 16 KiB boot section
- keep the application start fixed at byte address `0x4000`
- use `USART1` on the existing SignalSlinger serial pins, `PC0` TX and `PC1` RX
- stay in the bootloader when the front-panel switch is held at reset, when no valid app reset vector is present, or when the serial updater sends `U` during the startup window
- otherwise jump to the relocated application

The first implementation intentionally does not write flash. Page erase/write and host upload framing should be added after this skeleton builds and the entry/jump behavior is verified on hardware with UPDI recovery available.

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

## Provisioning Notes

Do not program bootloader fuses casually. The safe provisioning flow should remain:

1. build the bootloader
2. program the bootloader over UPDI
3. set `BOOTSIZE = 32` and `CODESIZE = 0`
4. build and program a relocated application
5. verify serial entry, timeout jump, and switch-held entry

UPDI must remain available as the recovery path throughout bootloader development.
