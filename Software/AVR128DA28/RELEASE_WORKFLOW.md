# Release Workflow

This document captures the standard release process for the AVR128DA28 firmware workspace.

## Branch Roles

- `main` is the stable release branch.
- `Development2` is the development prerelease branch.
- Before making any software changes, explicitly state the current git branch to the user.
- On `Development2`, do not automatically commit changes unless the user explicitly authorizes that commit.

## Standard Release Checklist

1. Confirm the current branch and announce it to the user before making changes.
2. Confirm the intended firmware version in `SignalSlinger/defs.h`.
3. Decide the release channel:
   - `main` for a stable release.
   - `Development2` for a prerelease.
4. For routine patch verification before handoff, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release
```

Optional: for dual-target `.hex` verification with SHA256 reporting, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\verify-firmware-hashes.ps1 -Configuration Release
```

5. Run the release preparation script:

```powershell
powershell -ExecutionPolicy Bypass -File .\prepare-release.ps1
```

6. Verify the generated `Release` assets exist:
   - `SignalSlinger-vX.Y-3.4.hex`
   - `SignalSlinger-vX.Y-3.5.hex`
7. Review `README.md` and confirm it matches the intended branch and release channel.
8. On `Development2`, leave changes uncommitted unless the user explicitly asks for a commit.
9. Before using `gh`, ensure the GitHub CLI environment is clean in this VM session:

```powershell
$env:Path = 'C:\Program Files\GitHub CLI;' + [Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [Environment]::GetEnvironmentVariable('Path','User')
$env:HTTP_PROXY=''
$env:HTTPS_PROXY=''
$env:ALL_PROXY=''
```

10. Create the GitHub release:
   - Use a prerelease on `Development2`.
   - Use a normal release on `main`.
11. Upload both hardware assets to the release.
12. Verify remotely:
   - the release page includes both `.hex` files
   - `main` README points to stable downloads
   - `Development2` README points to prerelease downloads

## Notes

- `build-firmware.ps1` is the standard local build entry point for patch verification and can also be reused by other scripts.
- `verify-firmware-hashes.ps1` builds both hardware targets, reports SHA256 hashes for the copied `.hex` files, and restores the original active hardware target afterward.
- Use `-OutputDir` or `-KeepArtifacts` with `verify-firmware-hashes.ps1` if you want to keep the copied comparison artifacts.
- The release-prep flow should build `Release`, not `Debug`.
- The expected asset naming pattern is `SignalSlinger-vX.Y-3.4.hex` and `SignalSlinger-vX.Y-3.5.hex`.
- `prepare-release.ps1` should restore the default hardware target after the dual-build process completes.

## Branch Merge Policy

- Use `.\merge-development2-into-main.ps1` for `Development2` to `main` merges instead of a raw `git merge`.
- The merge worktree must start clean so the renormalization step only stages merge content.
- The policy for this repository is `core.autocrlf=false` with `.gitattributes` controlling line endings.
- The merge helper also enforces `core.safecrlf=true` so Git rejects mixed or lossy line-ending conversions.
- The merge must be performed with `--no-commit`, then `git add --renormalize .` must run before the merge commit is created.
- If conflicts occur, resolve them first, rerun the helper, and only then create the merge commit.
- Review `git status` and `git diff --cached --stat` before finalizing the merge commit.
