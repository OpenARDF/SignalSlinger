# SignalSlinger EEPROM Map

This document describes the compact v2 EEPROM layout selected by initialization
marker `0x0200`. The target AVR128DA28 supplies 512 bytes of EEPROM, addressed
from `0x000` through `0x1FF`.

The C++ declarations and compile-time assertions remain authoritative:

- [`SignalSlinger/include/eeprommanager.h`](SignalSlinger/include/eeprommanager.h)
  defines the packed settings structure and pinned setting offsets.
- [`SignalSlinger/src/eeprommanager.cpp`](SignalSlinger/src/eeprommanager.cpp)
  defines the fixed region boundaries, migration, and journal operations.
- [`SignalSlinger/include/session_history.h`](SignalSlinger/include/session_history.h)
  defines the session-record format.
- [`SignalSlinger/include/hardware_fault_history.h`](SignalSlinger/include/hardware_fault_history.h)
  defines the hardware-fault-record format.

## Region map

| Decimal addresses | Hex addresses | Size | Purpose |
| ---: | ---: | ---: | --- |
| 0-138 | `0x000-0x08A` | 139 bytes | Active configuration settings |
| 139-159 | `0x08B-0x09F` | 21 bytes | Reserved for append-only settings growth |
| 160-369 | `0x0A0-0x171` | 210 bytes | Seven 30-byte session-history records |
| 370-449 | `0x172-0x1C1` | 80 bytes | Eight 10-byte hardware-fault records |
| 450-511 | `0x1C2-0x1FF` | 62 bytes | Unallocated future space |

Settings, session history, and hardware-fault history are independently bounded.
Log rotation cannot overwrite configuration, and adding a setting cannot silently
move either journal. The firmware rejects an oversized settings structure or an
overall region overflow at compile time.

## Configuration region

`EE_prom` is packed, so the stored representation contains no implicit alignment
padding. Multi-byte values use the AVR's little-endian representation.

| Decimal address | Hex address | Size | Type | Stored value |
| ---: | ---: | ---: | --- | --- |
| 0 | `0x000` | 2 | `uint16_t` | EEPROM initialization/layout marker |
| 2 | `0x002` | 4 | `time_t` | Event start epoch |
| 6 | `0x006` | 4 | `time_t` | Event finish epoch |
| 10 | `0x00A` | 22 | `char[22]` | Transmission pattern text |
| 32 | `0x020` | 22 | `char[22]` | Foxoring pattern text |
| 54 | `0x036` | 22 | `char[22]` | Station ID text |
| 76 | `0x04C` | 1 | `Fox_t` | Classic fox role |
| 77 | `0x04D` | 1 | `Fox_t` | Sprint fox role |
| 78 | `0x04E` | 1 | `Fox_t` | Foxoring fox role |
| 79 | `0x04F` | 1 | `uint8_t` | UTC offset |
| 80 | `0x050` | 2 | `uint16_t` | RF power |
| 82 | `0x052` | 1 | `uint8_t` | Station-ID code speed |
| 83 | `0x053` | 1 | `uint8_t` | Pattern code speed |
| 84 | `0x054` | 1 | `uint8_t` | Foxoring-pattern code speed |
| 85 | `0x055` | 2 | `int16_t` | Off-air seconds |
| 87 | `0x057` | 2 | `int16_t` | On-air seconds |
| 89 | `0x059` | 2 | `int16_t` | ID period seconds |
| 91 | `0x05B` | 2 | `int16_t` | Intra-cycle delay seconds |
| 93 | `0x05D` | 1 | `Event_t` | Event setting |
| 94 | `0x05E` | 4 | `uint32_t` | Active frequency |
| 98 | `0x062` | 4 | `uint32_t` | Low frequency bank |
| 102 | `0x066` | 4 | `uint32_t` | Medium frequency bank |
| 106 | `0x06A` | 4 | `uint32_t` | High frequency bank |
| 110 | `0x06E` | 4 | `uint32_t` | Beacon frequency bank |
| 114 | `0x072` | 4 | `float` | Battery-voltage threshold |
| 118 | `0x076` | 2 | `uint16_t` | RTC calibration |
| 120 | `0x078` | 4 | `int32_t` | Si5351 reference correction in ppb |
| 124 | `0x07C` | 1 | `uint8_t` | Days to run |
| 125 | `0x07D` | 1 | `int8_t` | Thermal-shutdown threshold in Celsius |
| 126 | `0x07E` | 1 | `uint8_t` | Thermal-shutdown enable marker |
| 127 | `0x07F` | 4 | `float` | Hottest-ever temperature |
| 131 | `0x083` | 1 | `uint8_t` | Function setting |
| 132 | `0x084` | 1 | `uint8_t` | Boost-regulator enable |
| 133 | `0x085` | 1 | `uint8_t` | External-battery-control enable |
| 134 | `0x086` | 1 | `uint8_t` | Device enabled |
| 135 | `0x087` | 4 | `uint32_t` | Saved RF calibration carrier frequency in Hz |
| 139-159 | `0x08B-0x09F` | 21 | Reserved | Append-only settings expansion |

The offset assertions in `eeprommanager.h` intentionally pin established
settings. A new setting must be appended at address 139 or later while remaining
inside the 160-byte settings region. Inserting a field among existing settings,
growing beyond address 159, or changing an established representation requires
an explicit EEPROM-map version and migration.

## Session-history region

The session journal contains seven rotating slots. Each slot occupies 30 bytes:

| Record offset | Size | Type | Stored value |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32_t` | Sequence number |
| 4 | 4 | `uint32_t` | Schedule identifier/time |
| 8 | 4 | `uint32_t` | Start time |
| 12 | 4 | `uint32_t` | Finish time |
| 16 | 4 | `uint32_t` | Record timestamp |
| 20 | 2 | `int16_t` | Temperature |
| 22 | 1 | `uint8_t` | Action |
| 23 | 1 | `uint8_t` | Reason |
| 24 | 1 | `uint8_t` | Flags |
| 25 | 1 | `uint8_t` | Thermal threshold |
| 26 | 1 | `uint8_t` | Validity magic (`0xA7`) |
| 27 | 1 | `uint8_t` | Record version |
| 28 | 2 | `uint16_t` | CRC-16 checksum |

The seven slot starts are `0x0A0`, `0x0BE`, `0x0DC`, `0x0FA`, `0x118`,
`0x136`, and `0x154`.

## Hardware-fault region

The hardware-fault log contains eight rotating slots. A fault is retained across
reset until the user clears the history with `HWF C`. Each slot occupies 10 bytes:

| Record offset | Size | Type | Stored value |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32_t` | Detection date/time, or zero when unavailable |
| 4 | 1 | `uint8_t` | Stable hardware-fault code |
| 5 | 1 | `uint8_t` | Flags, including time-valid |
| 6 | 1 | `uint8_t` | Validity magic (`0x48`) |
| 7 | 1 | `uint8_t` | Record version |
| 8 | 2 | `uint16_t` | CRC-16 checksum |

The eight slot starts are `0x172`, `0x17C`, `0x186`, `0x190`, `0x19A`,
`0x1A4`, `0x1AE`, and `0x1B8`.

## Legacy migration

Initialization markers `0x0131` through `0x0136` identify the former 288-byte
padded settings layout. On first boot of compact-v2 firmware, migration:

1. Reads the entire legacy settings structure and all seven legacy session slots
   into RAM before making an overlapping write.
2. Invalidates the EEPROM marker so an interrupted rewrite cannot be accepted as
   a valid compact-v2 image.
3. Copies supported settings to their compact addresses, initializes the saved
   calibration carrier to 3,600,000 Hz, and normalizes settings introduced after
   the detected legacy version.
4. Rewrites valid session records into the relocated session journal.
5. Clears the new hardware-fault region, for which no legacy records exist.
6. Commits marker `0x0200` last.

If migration is interrupted, firmware treats EEPROM as uninitialized and uses
safe defaults. SerialSlinger retains its pre-update settings backup until
post-reboot restoration has been verified, providing the host-side recovery path.

## Change rules

When this map changes:

1. Update the packed structures, region constants, offset/size assertions,
   migration, and this document together.
2. Preserve established offsets whenever the change fits in the reserved
   settings bytes.
3. Never grow a journal merely by moving the following region; change the map
   version and provide an explicit migration.
4. Keep validity markers and checksums in every journal-record format so partial
   EEPROM writes are rejected.
5. Run `just test`, `just docs-check`, and `just check`. Firmware behavior changes
   additionally require the applicable AVR builds.
