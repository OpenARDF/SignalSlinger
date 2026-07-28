# SignalSlinger AVR Build Environment

## macOS reference build

SignalSlinger firmware can be built on macOS without Microchip Studio. The
portable wrappers reproduce the checked-in Microchip Studio Release
configuration for the AVR128DA28:

- AVR-GCC 7.3.0;
- Atmel `AVR-Dx_DFP` 1.10.114;
- `-O1`, `-fpack-struct`, `-fshort-enums`, section garbage collection, and the
  include/source lists recorded by `SignalSlinger/Release/Makefile`.

Install the exact archived tools once:

```sh
just avr-setup-macos
```

The installer downloads the official archives, checks their pinned SHA-256
digests before extraction, and places them under ignored
`tmp/avr-tools/`. It is idempotent and refuses to replace an unexpected or
incomplete installation.

Verify the environment without compiling:

```sh
just avr-doctor
```

The doctor must report `reference-version-match`. A different compiler or
device pack is rejected unless `AVR_ALLOW_VERSION_MISMATCH=1` is explicitly set
for an exploratory, non-release build.

The wrappers also accept explicit `AVR_TOOLCHAIN_ROOT` and `AVR_DFP_ROOT`
locations. Those variables take precedence over the repo-local installation.

## Build commands

```sh
# Normal Release build using the target selected in defs.h
just avr-build

# Normal Release builds for both supported hardware revisions
just avr-dual-build

# Resident bootloader plus both applications relocated to 0x2000
just avr-boot-chain-build

# Repeat the active build twice and compare every artifact
just avr-repeatability

# Run repository/tooling checks without building firmware
just check

# Build and validate both complete release packages
just avr-release-packages

# Run the complete deterministic release-candidate gate
just release-preflight
```

Explicit target recipes are also available:

```sh
just avr-build-hw 3.4
just avr-build-hw 3.5
just avr-relocated-build-hw 3.4
just avr-relocated-build-hw 3.5
just avr-repeatability-hw 3.5
just avr-relocated-repeatability-hw 3.5
just avr-bootloader-repeatability
```

Run `just --list` for the complete command list.

For the existing macOS PowerShell/pymcuprog hardware path, first build the boot
chain, then check its prerequisites without touching a target:

```sh
just avr-boot-chain-build
just avr-programmer-prereqs
```

With a programmer and SignalSlinger connected, `just avr-probe` performs the
existing no-write programmer/device check. Entering UPDI programming mode may
reset the transmitter even though the recipe does not write it. Destructive
provisioning remains in `provision-bootloader.ps1`, with its explicit fuse-write
confirmation switches; it is intentionally not reduced to an easy Just recipe.

## Build outputs and evidence

All portable build products are written beneath ignored `tmp/` directories.
Each build starts from an empty output directory and produces
`build-evidence.json` containing:

- compiler and device-pack versions;
- reference or exploratory status;
- hardware target and application start address;
- AVR size output and linked `.text` size;
- every compiler warning;
- byte size and SHA-256 for every generated artifact.

Normal application builds use `tmp/avr-release-*`. Relocated builds use
`tmp/avr-release-relocated-*`, and bootloader builds use
`tmp/avr-bootloader-release/`.

The wrapper rejects an incorrect application start, application overflow beyond
128 KiB flash, and a bootloader larger than the reserved 8 KiB section.

Complete packages are written beneath ignored `release-packages/` directories.
Each hardware package contains the normal SerialSlinger update HEX, a combined
bootloader-and-application first-install HEX, the bootloader setup helper,
workshop PowerShell tools, a machine-readable release-info manifest, SHA-256
checksums, a plain-language README, and a ZIP containing that exact set.

`just avr-release-package-repeatability` generates both packages twice and
requires every file, including each ZIP, to be byte-identical. Package
timestamps come from the source commit rather than wall-clock time. The
release-info manifest records whether the firmware worktree was dirty; release
publication still requires a clean frozen commit recorded in the release
checklist.

## Repository and release gates

The portable repository checks mirror the applicable FlexFox workflow:

```sh
just doctor
just docs-check
just policy-check
just test
just check
just secrets
```

Validate a release-specific evidence checklist with:

```sh
just release-version-prepare 2.0.3 stable
just release-state-check
just release-notes-check ../../release-evidence/release-checklist-v2.0.3.json
just release-publication-check ../../release-evidence/release-checklist-v2.0.3.json candidate
just release-checklist ../../release-evidence/release-checklist-v2.0.3.json release
just release-checklist ../../release-evidence/release-checklist-v2.0.3.json final
```

`just release-preflight` validates the current version's release notes, runs
the non-firmware checks and secret scan, and requires normal and relocated
repeatability for HW-3.4 and HW-3.5, bootloader repeatability, and
release-package repeatability. It first verifies that the firmware and README
use the next unused patch version and that local semantic tags agree with
GitHub. It does not replace physical programming,
serial-update, recovery, or live `INF` verification on representative hardware.
The publication gate also requires versioned release notes with substantive
user-visible and stability/reliability sections. GitHub publication must use
that checked file with `gh release create --notes-file`; after publication,
`just release-notes-remote-check <checklist>` requires the remote body to match
the file exactly.

For stable integration, `just release-integration-worktree 2.0.3` creates a
clean main worktree under `/private/tmp` and stages the renormalized
Development2 merge without committing it. This keeps unrelated changes in the
normal checkout, including KiCad edits, outside the release merge.

## Pinned archives

- Microchip AVR 8-bit GNU Toolchain 3.7.0 for macOS:
  `avr8-gnu-toolchain-osx-3.7.0.518-darwin.any.x86_64.tar.gz`,
  SHA-256
  `378b210cc82dc06599b5a45dede0ed188a9e87f2de045f12cf3547554edd6ec8`.
- Atmel AVR-Dx_DFP 1.10.114:
  `Atmel.AVR-Dx_DFP.1.10.114.atpack`, SHA-256
  `8eee80fe5cd01c54edd1959fd091043156156b4c05f84c7e5ea6e590714096fe`.

The compiler is an x86_64 macOS executable and runs through Rosetta on Apple
silicon.

## Windows compatibility

The existing PowerShell/Microchip Studio build scripts remain available on
Windows. The portable wrappers are a separate build path and do not modify the
generated Studio Makefiles or their normal output directories. Release
qualification should compare Mac and Windows HEX, EEPROM, warnings, and memory
totals whenever the toolchain configuration itself changes.
