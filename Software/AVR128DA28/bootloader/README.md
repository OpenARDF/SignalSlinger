# SignalSlinger Bootloader

This directory contains the first-stage SignalSlinger AVR128DA28 bootloader work.

Current scope:

- reserve a 16 KiB boot section
- keep the application start fixed at byte address `0x4000`
- use `USART1` on the existing SignalSlinger serial pins, `PC0` TX and `PC1` RX
- latch SignalSlinger power on with `POWER_ENABLE`/`PA3` before waiting for serial input
- turn both front-panel LEDs on during the bootloader startup window for immediate power-on feedback, blink them at the wake-authorization cadence when the power button is held, then pass the held-button state to the app at handoff
- stay in the bootloader when no valid app reset vector is present, when the serial updater sends `U` during the startup window, or when the front-panel switch is held during a non-power reset
- otherwise jump to the relocated application

The bootloader now accepts a deliberately small page programming protocol. It only erases or writes full, page-aligned APPCODE pages and rejects any frame whose address would touch the boot section.

The running application still accepts the `UPD` command at the normal SignalSlinger serial rate. After the app resets, the bootloader runs the programming protocol at `115200` baud. On the current SignalSlinger firmware image this writes the app in about 40-45 seconds, or about a minute with default serial page-CRC verification, which is a useful improvement without pushing the serial link hard.

On SignalSlinger hardware, the front-panel switch is also the momentary power-on source. A normal cold power-on therefore starts with that switch held. The bootloader treats POR/BOR resets specially: it latches `PA3` high immediately so the board stays powered, turns both LEDs on for immediate feedback, blinks both LEDs at the wake-authorization cadence while the button remains held, ignores switch-held bootloader entry for that cold-start case, and then jumps to the app unless serial `U` arrives or the app reset vector is missing. At app handoff, it records whether the power button is still held in `GPR0`; the app consumes that marker after its GPIO initialization so it can preserve startup LED feedback only for real held-button power-on. This lets the application take over the normal power latch/off behavior and LED policy without showing a long solid-LED hold when external power appears without a button press.

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

That helper uses a temporary makefile and injects `-Wl,--section-start=.text=0x4000`; it does not alter the normal `SignalSlinger/Release/Makefile`. It removes the previous final firmware outputs before linking and then verifies the generated map so stale non-relocated output cannot be mistaken for a bootloader-ready app image.

## Provisioning Script

Use the guarded provisioner to build, merge, program, verify, and validate a bootloader-enabled unit:

```powershell
powershell -ExecutionPolicy Bypass -File .\provision-bootloader.ps1 -Port COM6
```

Check programming PC prerequisites without touching target hardware:

```powershell
powershell -ExecutionPolicy Bypass -File .\provision-bootloader.ps1 -CheckPrereqs -SkipBuild -SkipSerialValidation
```

The script creates `tmp\SignalSlinger-bootloader-combined.hex` from the bootloader HEX and relocated app HEX, chip-erases the target, programs and verifies the combined flash image, reads fuses, verifies `CODESIZE = 0x00` and `BOOTSIZE = 0x20`, then runs the bootloader serial protocol test.

Fuse writes are deliberately opt-in:

```powershell
powershell -ExecutionPolicy Bypass -File .\provision-bootloader.ps1 -Port COM6 -ProgramFuses -ConfirmFuseWrite
```

Without those two switches, the script reads and checks fuses but does not change them.

On Windows, the provisioner uses Microchip Studio `atprogram` by default. On macOS or Windows, it can use `pymcuprog`:

```powershell
pwsh ./provision-bootloader.ps1 -Backend Pymcuprog -SkipBuild -BootloaderHexPath ./bootloader/Release/SignalSlingerBootloader.hex -ApplicationHexPath ./SignalSlinger/Release/SignalSlinger.hex
```

The current build helpers are Windows/Microchip Studio centered. On macOS, use prebuilt bootloader and relocated application HEX files with `-SkipBuild`, unless the AVR GCC build tooling has been ported locally.

Required programming PC software:

- Windows build/provision path: PowerShell, Microchip Studio 7 with the AVR-Dx device pack and AVR GCC toolchain, Atmel-ICE drivers, and a USB serial driver for the SignalSlinger serial adapter.
- Windows or macOS provision-only path: PowerShell 7, Python, `pymcuprog`, Atmel-ICE USB access, and prebuilt bootloader/application HEX files. `pymcuprog` is installed with `python -m pip install pymcuprog`; use a Python version with available `hidapi` wheels for the host architecture.
- Bench validation path: Atmel-ICE on UPDI plus a USB serial adapter connected to SignalSlinger USART1.

## Release Package

Build the files that should be attached to a GitHub release:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-release-package.ps1
```

The package is written under `release-packages\...` by default and uses names meant to be understandable outside the firmware bench:

- `SignalSlinger-Update-...hex`: the file SerialSlinger should use for normal software updates
- `SignalSlinger-First-Install-...hex`: the file workshop setup tools should use for a new board with a programmer
- `SignalSlinger-Bootloader-...hex`: the helper file used by workshop setup tools
- `SignalSlinger-Release-Info-...json`: the information SerialSlinger reads automatically
- `SignalSlinger-Checksums-...txt`: optional file-integrity checks
- `README-SignalSlinger-...txt`: short plain-language notes for the release folder

The release-info JSON keeps the unavoidable details for software: board version, update speed, page size, app start address, bootloader version, protocol version, and SHA-256 hashes. Normal users should not need those details; SerialSlinger can read them and choose the right update file in the background.

## Serial Update Frames

All multi-byte fields are little-endian. CRC is CRC-16/CCITT-FALSE initialized to `0xFFFF` and covers the command byte plus the frame body, but not the transmitted CRC bytes.

- `?`: print bootloader information
- `U`: enter or confirm bootloader update mode
- `R`: jump to the app when the app reset vector is programmed
- `E <addr:u32> <crc:u16>`: erase one 512-byte page
- `W <addr:u32> <payload:512 bytes> <crc:u16>`: write one erased 512-byte page
- `C <addr:u32> <crc:u16>`: report CRC-16/CCITT-FALSE of one 512-byte page as `OK crc 0xAAAAAAAA CCCC`

`addr` must be page-aligned and the complete page must be inside `0x04000` through `0x1FFFF`. The bootloader responds with `OK erase`, `OK write`, `OK crc ...`, or `ERR ...`. USART framing, parity, or overflow faults are reported as `ERR serial XX`.

The `?` response is intentionally machine-readable for host programmers:

```text
SignalSlinger BL0.10 proto=1 app=0x4000 page=512 flash=131072 baud=115200 boot=32 cmds=U,R,?,E,W,C
```

During a firmware update, the bootloader toggles red as erase/write flash operations are accepted and completed, and toggles green as write payload bytes are received. Rejected frames and NVM/serial errors leave red on and green off until later update activity changes the indication.

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

The updater defaults to `-AppBaud 9600` for the `UPD` command and `-BootBaud 115200` for the bootloader frames. The legacy `-Baud` parameter is still accepted as an alias for `-BootBaud`.

For bench validation with Atmel-ICE attached, add `-VerifyWithUpdi` to verify the programmed app flash after the serial update:

```powershell
powershell -ExecutionPolicy Bypass -File .\update-firmware-serial.ps1 -Port COM6 -RequestBootloaderFromApp -VerifyWithUpdi
```

The updater parses the relocated Intel HEX file, rejects records outside APPCODE, erases the reset-vector page first, writes all other pages, then writes the reset-vector page last. By default it asks the bootloader for a CRC of each programmed page and compares it with the image data, giving serial-only verification without Atmel-ICE. Use `-SkipSerialVerify` only for bench timing or protocol debugging.

If an update is interrupted after the reset-vector page is erased, the application will not start. That is intentional: on the next reset, the missing app vector keeps the bootloader resident so the unit can be restored with:

```powershell
powershell -ExecutionPolicy Bypass -File .\update-firmware-serial.ps1 -Port COM6 -SkipBuild -NoReset -VerifyWithUpdi
```

## Bench Protocol Tests

With Atmel-ICE and the USB serial adapter attached, run the bootloader protocol hardening test from the app `UPD` path:

```powershell
powershell -ExecutionPolicy Bypass -File .\test-bootloader-serial.ps1 -Port COM6 -RequestBootloaderFromApp
```

Run the same test through UPDI reset plus serial `U` entry:

```powershell
powershell -ExecutionPolicy Bypass -File .\test-bootloader-serial.ps1 -Port COM6
```

The test uses page `0x1FE00` as scratch, validates good erase/write/CRC behavior, rejects bad CRCs, rejects unaligned, boot-section, and past-flash addresses, verifies truncated-frame timeout handling, confirms the bootloader remains responsive afterward, and erases the scratch page before returning to the app.

For repeatability testing, run multiple full updates from the app `UPD` path:

```powershell
powershell -ExecutionPolicy Bypass -File .\test-bootloader-repeatability.ps1 -Port COM6 -Count 3
```

By default each repeatability run verifies every page over serial CRC, verifies the programmed application over UPDI, and reports programming and wall-clock time. The script writes timestamped run artifacts under `tmp\bootloader-repeatability\...`: one log per update plus `summary.csv` and `summary.json` with the git commit, HEX path, HEX SHA-256, baud rates, verification settings, and per-run timings.

Use `-OutputDir <path>` to choose a durable evidence folder. Use `-SkipSerialVerify` only when measuring raw programming time; use `-NoVerify` only when Atmel-ICE is not attached.

## Provisioning Notes

Do not program bootloader fuses casually. The safe provisioning flow should remain:

1. build the bootloader
2. program the bootloader over UPDI
3. verify fuse backups exist
4. set `BOOTSIZE = 32` and `CODESIZE = 0`
5. verify `BOOTSIZE = 0x20` and `CODESIZE = 0x00`
6. build and program a relocated application
7. run `test-bootloader-serial.ps1` through both app `UPD` and UPDI reset entry
8. run one full serial update with `-VerifyWithUpdi`
9. confirm the application responds at the normal app baud

The normal production firmware makefile should remain unmodified while the relocated build path is still being proven.

## Recovery Checklist

If a unit does not start the app after an attempted update:

1. connect the USB serial adapter at `115200` baud
2. send `?` and confirm the `SignalSlinger BL...` banner
3. if there is no response, reset with UPDI while sending `U`
4. run the serial updater with `-NoReset -VerifyWithUpdi`
5. confirm the app responds at `9600` baud

If serial recovery does not respond, use UPDI to reprogram the bootloader and relocated app, then re-check `BOOTSIZE` and `CODESIZE`. Do not change fuses unless UPDI communication is known-good.

Reference validation sequence before broader use:

1. `powershell -ExecutionPolicy Bypass -File .\build-bootloader.ps1 -Configuration Release`
2. `powershell -ExecutionPolicy Bypass -File .\build-relocated-firmware.ps1 -Configuration Release`
3. `powershell -ExecutionPolicy Bypass -File .\test-bootloader-serial.ps1 -Port COM6 -RequestBootloaderFromApp`
4. `powershell -ExecutionPolicy Bypass -File .\test-bootloader-serial.ps1 -Port COM6`
5. `powershell -ExecutionPolicy Bypass -File .\test-bootloader-repeatability.ps1 -Port COM6 -Count 3`

UPDI must remain available as the recovery path throughout bootloader development.
