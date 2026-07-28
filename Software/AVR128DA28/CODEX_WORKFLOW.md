# Codex Workflow

## Everyday Expectations

- Before making any software changes, explicitly state the current git branch to the user.
- Keep this branch announcement in the commentary/update message that comes before the edit.
- Prefer `rg` for text searches and `rg --files` for file discovery when working in this repo; use recursive PowerShell searches only as a fallback when `rg` is unavailable or blocked in the current shell.
- On `Development2`, do not automatically commit changes; leave them uncommitted for the user to review and commit.
- For `Development2` to `main` branch syncs, use `.\merge-development2-into-main.ps1` so the merge always includes the line-ending renormalization step before commit.

## Build And Verification

- Use `.\build-firmware.ps1` as the standard patch-verification path instead of reconstructing `make.exe` commands ad hoc.
- On macOS, use `just avr-build` as the equivalent standard patch-verification path. Run `just avr-setup-macos` once on a new checkout and require `just avr-doctor` to report `reference-version-match`.
- Use `just avr-dual-build` for both normal hardware targets and `just avr-boot-chain-build` for the resident bootloader plus both applications relocated to `0x2000`.
- Start repository work with `just --list`; use `just check` for the portable non-firmware gate and `just release-preflight` for the complete Mac release-candidate gate.
- Use `just avr-release-packages` to create and independently validate both hardware-specific release bundles. Do not reconstruct package manifests, checksums, first-install images, or ZIPs by hand.
- Use `just release-version-prepare <version> <stable|prerelease>` for the guarded patch-version increment, README update, remote-tag collision check, and release-checklist creation. Do not change release version sources independently.
- Use `just release-state-check` before release preflight. Local semantic tags must match GitHub, and the candidate tag must be unused.
- Treat `release-notes/vX.Y.Z.md` as a required, versioned release artifact. It must summarize salient user-observable changes and significant stability/reliability improvements since the checklist's previous release, name all four release assets, and link the exact GitHub comparison.
- Run `just release-publication-check <checklist> <candidate|release>` before GitHub publication. Always create the GitHub release with `gh release create --notes-file release-notes/vX.Y.Z.md`; generated notes are not an acceptable final release body.
- After publication, run `just release-notes-remote-check <checklist>` and record evidence that the GitHub body exactly matches the checked file.
- Use `just release-integration-worktree <version>` to stage the Development2-to-main merge in a clean `/private/tmp` worktree without touching unrelated KiCad changes in the normal checkout.
- Release-specific checklists live under repo-root `release-evidence/` until publication is complete. The normal staging helper excludes that directory; commit the final evidence after the tag is fixed, without moving the tag.
- Use `just avr-programmer-prereqs` for a no-hardware setup check and `just avr-probe` for a no-write connected-target check. Do not add a shortcut around the explicit fuse confirmations in `provision-bootloader.ps1`.
- Portable Mac build artifacts and `build-evidence.json` files belong only under the ignored `tmp/avr-*` directories.
- Portable release packages belong only under the ignored `release-packages/` directory and must be published from a clean frozen commit even though local package generation records and permits a dirty development worktree.
- After any executable firmware source or configuration change, run `powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release` before handing the patch back unless the user explicitly asks not to build or the environment blocks it.
- When working on macOS, the preceding requirement is satisfied by the matching `just` recipe rather than the Windows-only PowerShell command.
- For comment-only, documentation-only, or ignore-only changes, a firmware build is optional.
- When a no-behavior-change pass needs stronger proof, run `powershell -ExecutionPolicy Bypass -File .\verify-firmware-hashes.ps1 -Configuration Release` to rebuild both hardware targets and report SHA256 hashes.
- On macOS, use `just avr-repeatability` or `just avr-repeatability-hw <target>` for byte-for-byte repeatability evidence.
- Use `..\..\prepare-release.ps1` only for release preparation; it is a repo-root script, not the routine patch-verification path.

## Generated And Hand-Maintained Files

- Hand-maintained firmware files include `main.cpp`, `defs.h`, most files under `SignalSlinger\src`, most files under `SignalSlinger\include`, the PowerShell helper scripts, and the README/workflow documents.
- Treat the following as generated or vendor-managed surfaces that should not be casually reorganized or reformatted: `atmel_start.cpp`, `atmel_start.h`, `include\atmel_start_pins.h`, `driver_isr.cpp`, `Config\*`, `Debug\Makefile`, `Release\Makefile`, `SignalSlinger.cppproj`, `SignalSlinger.componentinfo.xml`, `modules\*`, and `utils\*`.
- If a targeted fix in a generated or vendor-managed file is unavoidable, keep the existing license block intact, document why the edit was needed, and assume a future START regeneration may overwrite it.
