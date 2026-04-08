# Codex Workflow

- Before making any software changes, explicitly state the current git branch to the user.
- Keep this branch announcement in the commentary/update message that comes before the edit.
- Prefer `rg` for text searches and `rg --files` for file discovery when working in this repo; use recursive PowerShell searches only as a fallback when `rg` is unavailable or blocked in the current shell.
- After any firmware source change, run `powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release` before handing the patch back unless the user explicitly asks not to build or the environment blocks it.
- Use `.\build-firmware.ps1` as the standard patch-verification path instead of reconstructing `make.exe` commands ad hoc.
- On `Development2`, do not automatically commit changes; leave them uncommitted for the user to review and commit.
- For `Development2` to `main` branch syncs, use `.\merge-development2-into-main.ps1` so the merge always includes the line-ending renormalization step before commit.
