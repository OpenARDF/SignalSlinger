# Codex Workflow

## Everyday Expectations

- Before making any software changes, explicitly state the current git branch to the user.
- Keep this branch announcement in the commentary/update message that comes before the edit.
- Prefer `rg` for text searches and `rg --files` for file discovery when working in this repo; use recursive PowerShell searches only as a fallback when `rg` is unavailable or blocked in the current shell.
- On `Development2`, do not automatically commit changes; leave them uncommitted for the user to review and commit.
- For `Development2` to `main` branch syncs, use `.\merge-development2-into-main.ps1` so the merge always includes the line-ending renormalization step before commit.

## Build And Verification

- Use `.\build-firmware.ps1` as the standard patch-verification path instead of reconstructing `make.exe` commands ad hoc.
- After any executable firmware source or configuration change, run `powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release` before handing the patch back unless the user explicitly asks not to build or the environment blocks it.
- For comment-only, documentation-only, or ignore-only changes, a firmware build is optional.
- When a no-behavior-change pass needs stronger proof, run `powershell -ExecutionPolicy Bypass -File .\verify-firmware-hashes.ps1 -Configuration Release` to rebuild both hardware targets and report SHA256 hashes.
- Use `..\..\prepare-release.ps1` only for release preparation; it is a repo-root script, not the routine patch-verification path.

## Generated And Hand-Maintained Files

- Hand-maintained firmware files include `main.cpp`, `defs.h`, most files under `SignalSlinger\src`, most files under `SignalSlinger\include`, the PowerShell helper scripts, and the README/workflow documents.
- Treat the following as generated or vendor-managed surfaces that should not be casually reorganized or reformatted: `atmel_start.cpp`, `atmel_start.h`, `include\atmel_start_pins.h`, `driver_isr.cpp`, `Config\*`, `Debug\Makefile`, `Release\Makefile`, `SignalSlinger.cppproj`, `SignalSlinger.componentinfo.xml`, `modules\*`, and `utils\*`.
- If a targeted fix in a generated or vendor-managed file is unavoidable, keep the existing license block intact, document why the edit was needed, and assume a future START regeneration may overwrite it.
