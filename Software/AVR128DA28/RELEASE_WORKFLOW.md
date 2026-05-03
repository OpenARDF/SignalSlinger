# Release Workflow

This document captures the standard release process for the AVR128DA28 firmware workspace.

Unless stated otherwise, commands below assume the current directory is `Software/AVR128DA28`.

## Branch Roles

- `main` is the stable release branch.
- `Development2` is the active development release branch and may host either prereleases or normal releases when explicitly requested.
- Before making any software changes, explicitly state the current git branch to the user.
- On `Development2`, do not automatically commit changes unless the user explicitly authorizes that commit.

## Standard Release Checklist

1. Confirm the current branch and announce it to the user before making changes.
2. Confirm the working tree is clean or that any existing changes are intentionally excluded from the release work:

```powershell
git status --short
```

3. Confirm the intended firmware version in `SignalSlinger/defs.h` and make sure the repo-root `README.md` references the same asset names for the chosen branch.
4. Decide the release channel:
   - `main` for a stable release.
   - `Development2` for a development-branch release, using either a prerelease or a normal release as explicitly requested.
5. For routine patch verification before handoff, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release
```

Optional: for dual-target `.hex` verification with SHA256 reporting, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\verify-firmware-hashes.ps1 -Configuration Release
```

6. Run the release preparation script from the repo root:

```powershell
powershell -ExecutionPolicy Bypass -File ..\..\prepare-release.ps1
```

7. Verify the generated `Release` assets exist:
   - `SignalSlinger-vX.Y-3.4.hex`
   - `SignalSlinger-vX.Y-3.5.hex`
8. Review `README.md` and confirm it matches the intended branch and release channel.
9. On `Development2`, leave changes uncommitted unless the user explicitly asks for a commit.
10. Before using `gh`, ensure the GitHub CLI environment is clean in this VM session:

```powershell
$env:Path = 'C:\Program Files\GitHub CLI;' + [Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [Environment]::GetEnvironmentVariable('Path','User')
$env:HTTP_PROXY=''
$env:HTTPS_PROXY=''
$env:ALL_PROXY=''
```

11. Draft user-readable GitHub release notes that cover the changes since the previous release.
   - Match the style of `v1.2.1`: a short version introduction, two to four plain-language paragraphs about user-visible changes, an overall summary sentence, and a `Full Changelog` compare link.
   - Do not publish GitHub's generated PR summary as the final release body. Use generated notes only as source material for the user-readable draft.
12. Create the GitHub release:
   - Use a prerelease or a normal release on `Development2` according to the requested release channel.
   - Use a normal release on `main` unless a prerelease is explicitly requested.
   - Prefer `--notes-file` or edit the release immediately after creation so the published body uses the drafted user-readable notes.
13. Upload both hardware assets to the release.
14. Verify remotely:
   - the release page includes both `.hex` files
   - the release notes are user-readable and summarize the changes since the previous release
   - `main` README points to stable downloads
   - `Development2` README points to the intended development-branch downloads

## Notes

- `build-firmware.ps1` is the standard local build entry point for patch verification and can also be reused by other scripts.
- `verify-firmware-hashes.ps1` builds both hardware targets, reports SHA256 hashes for the copied `.hex` files, and restores the original active hardware target afterward.
- Use `-OutputDir` or `-KeepArtifacts` with `verify-firmware-hashes.ps1` if you want to keep the copied comparison artifacts.
- `prepare-release.ps1` lives at the repository root and updates the repo-root `README.md` unless you pass `-SkipReadmeUpdate`.
- The release-prep flow should build `Release`, not `Debug`.
- The expected asset naming pattern is `SignalSlinger-vX.Y-3.4.hex` and `SignalSlinger-vX.Y-3.5.hex`.
- `prepare-release.ps1` should restore the default hardware target after the dual-build process completes.
- The generated `.hex` files are release artifacts, not tracked source files; upload them to GitHub releases rather than committing them.

## Branch Merge Policy

- Use `.\merge-development2-into-main.ps1` for `Development2` to `main` merges instead of a raw `git merge`.
- The merge worktree must start clean so the renormalization step only stages merge content.
- The policy for this repository is `core.autocrlf=false` with `.gitattributes` controlling line endings.
- The merge helper also enforces `core.safecrlf=true` so Git rejects mixed or lossy line-ending conversions.
- The merge must be performed with `--no-commit`, then `git add --renormalize .` must run before the merge commit is created.
- If conflicts occur, resolve them first, rerun the helper, and only then create the merge commit.
- Review `git status` and `git diff --cached --stat` before finalizing the merge commit.
