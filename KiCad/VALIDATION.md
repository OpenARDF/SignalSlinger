# KiCad Validation

Run the hardware-design gate from `Software/AVR128DA28`:

```sh
just kicad-check
just spice-check
```

The schematic currently passes KiCad 9.0.4 ERC with zero errors and zero
warnings. The PCB predates this gate and has 75 inherited DRC findings: 11
`hole_clearance` errors, 22 `lib_footprint_issues` warnings, and 42
`lib_footprint_mismatch` warnings. The normalized KiCad 9.0.4 DRC report has
SHA-256
`d7d1d6402bb9d74888b9200489501d575b2098d69caa2997805e6622305bc9ab`.

That exact report is the reviewed inherited exception. `just kicad-check` fails
if ERC reports any finding, if KiCad emits an unwaived diagnostic, or if the
normalized DRC report changes in any way. Any intentional DRC change must be
reviewed, explained here, and replace the hash and finding summary together.

`just spice-check` runs every `.cir`, `.sp`, and `.spice` deck under `KiCad/`
with warnings treated as failures. It reports an explicit pass when the
repository contains no simulation decks.
