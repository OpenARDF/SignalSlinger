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
4. Run the release preparation script:

```powershell
powershell -ExecutionPolicy Bypass -File .\prepare-release.ps1
```

5. Verify the generated `Release` assets exist:
   - `SignalSlinger-vX.Y-3.4.hex`
   - `SignalSlinger-vX.Y-3.5.hex`
6. Review `README.md` and confirm it matches the intended branch and release channel.
7. On `Development2`, leave changes uncommitted unless the user explicitly asks for a commit.
8. Before using `gh`, ensure the GitHub CLI environment is clean in this VM session:

```powershell
$env:Path = 'C:\Program Files\GitHub CLI;' + [Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [Environment]::GetEnvironmentVariable('Path','User')
$env:HTTP_PROXY=''
$env:HTTPS_PROXY=''
$env:ALL_PROXY=''
```

9. Create the GitHub release:
   - Use a prerelease on `Development2`.
   - Use a normal release on `main`.
10. Upload both hardware assets to the release.
11. Verify remotely:
   - the release page includes both `.hex` files
   - `main` README points to stable downloads
   - `Development2` README points to prerelease downloads

## Notes

- The release-prep flow should build `Release`, not `Debug`.
- The expected asset naming pattern is `SignalSlinger-vX.Y-3.4.hex` and `SignalSlinger-vX.Y-3.5.hex`.
- `prepare-release.ps1` should restore the default hardware target after the dual-build process completes.
