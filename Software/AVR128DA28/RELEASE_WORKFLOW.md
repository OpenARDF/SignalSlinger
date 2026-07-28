# Release Workflow

This document captures the standard release process for the AVR128DA28 firmware workspace.

Unless stated otherwise, commands below assume the current directory is `Software/AVR128DA28`.

## Branch Roles

- `main` is the stable release branch.
- `Development2` is the active development release branch and may host either prereleases or normal releases when explicitly requested.
- Before making any software changes, explicitly state the current git branch to the user.
- On `Development2`, do not automatically commit changes unless the user explicitly authorizes that commit.

## Standard Release Checklist

Normal releases increment the patch field from the latest semantic release on
GitHub. For example, if `v2.0.2` is the latest origin release, prepare `v2.0.3`.
Do not reuse an existing version or move a published tag.

`just release-version-prepare` updates `SignalSlinger/defs.h`, updates both
repo-root README asset names, creates a tracked versioned release-notes file
under `release-notes/`, and creates a release-specific checklist under
`release-evidence/`. The notes travel with the tagged source and are the sole
source for the GitHub release body. The checklist remains outside the tagged
source until publication is complete, so it can record the exact tag commit
without creating a self-referential commit hash. The final evidence file is
committed after the release tag is fixed.

Validate the untouched template, then run the release-specific checklist guard
at candidate, release, and final publication boundaries:

```sh
just release-checklist release-checklist-template.json template
just release-checklist ../../release-evidence/release-checklist-vX.Y.Z.json candidate
just release-checklist ../../release-evidence/release-checklist-vX.Y.Z.json release
just release-checklist ../../release-evidence/release-checklist-vX.Y.Z.json final
```

Record evidence without hand-editing JSON:

```sh
just release-checklist-done <file> <item-id> "<concrete evidence>"
just release-checklist-skip <file> <item-id> "<requester>" "<explicit waiver reason>"
```

1. Confirm the current branch and announce it to the user before making changes.
2. Confirm the working tree is clean or that any existing changes are intentionally excluded from the release work:

```powershell
git status --short
```

3. Refresh branches and tags from GitHub. This must finish without a rejected
   tag update:

```sh
git fetch origin --prune --tags
```

If a local semantic tag disagrees with GitHub, inspect both object IDs, replace
only the stale local reference from `origin`, and rerun the fetch. Never
force-push or otherwise move a published release tag.

4. Decide the release channel:
   - `main` for a stable release.
   - `Development2` for a development-branch release, using either a prerelease or a normal release as explicitly requested.
5. Prepare the next patch version. For a stable release after `v2.0.2`:

```sh
just release-version-prepare 2.0.3 stable
```

The recipe fails unless local semantic tags match GitHub, `v2.0.3` is absent
locally and remotely, and `2.0.3` is exactly the next patch after the latest
origin release. Review the resulting changes:

- `SignalSlinger/defs.h`: `SW_REVISION`
- repo-root `README.md`: both hardware update filenames
- `release-notes/v2.0.3.md`: the versioned GitHub release body
- `release-evidence/release-checklist-v2.0.3.json`: release metadata

The bootloader version is independent and changes only when bootloader
executable behavior or its protocol changes.

6. Commit and push the versioned release source on `Development2` when the user
   explicitly authorizes it. The standard non-KiCad staging helper also excludes
   `release-evidence/`, which remains local until post-release evidence is
   committed. Put the resulting full 40-character commit ID in the checklist's
   `sourceCommit`, then run:

```sh
just release-state-check
```

7. On macOS, run the complete release-candidate preflight:

```sh
just release-preflight
```

This validates the current version's release-notes content, runs the repository
checks and secret scan, then requires deterministic normal and relocated builds
for HW-3.4 and HW-3.5, a deterministic bootloader, and byte-identical complete
release packages. The package builder independently validates manifest hashes,
ZIP membership, and flash geometry.

For routine patch verification rather than release preparation, continue to use
`just avr-build` or `just avr-dual-build`.

On Windows, the established equivalents remain:

```powershell
powershell -ExecutionPolicy Bypass -File .\build-firmware.ps1 -Configuration Release
powershell -ExecutionPolicy Bypass -File .\verify-firmware-hashes.ps1 -Configuration Release
```

8. Build the complete hardware-specific release packages:

```sh
just avr-release-packages
```

The validated packages are written beneath:

- `release-packages/SignalSlinger-X.Y.Z-HW-3.4`
- `release-packages/SignalSlinger-X.Y.Z-HW-3.5`

For GitHub, upload each unzipped `SignalSlinger-Update-...hex` plus its matching
`SignalSlinger-...-Release-Files.zip`.

On Windows, `build-all-release-packages.ps1` and
`validate-release-package.ps1` remain supported.

9. Verify both packages report `sourceTreeDirty: false`, the recorded full
   commit is the frozen release commit, and all build profiles report
   `reference-version-match` with zero warnings.
10. Complete representative HW-3.4 and HW-3.5 programming, serial update,
    interrupted-update recovery, rollback, and live `INF` version checks.
11. Review `README.md` and confirm it matches the intended branch and release channel.
12. On `Development2`, leave changes uncommitted unless the user explicitly asks for a commit.
13. Before using `gh`, ensure the GitHub CLI environment is clean in this VM session:

```powershell
$env:Path = 'C:\Program Files\GitHub CLI;' + [Environment]::GetEnvironmentVariable('Path','Machine') + ';' + [Environment]::GetEnvironmentVariable('Path','User')
$env:HTTP_PROXY=''
$env:HTTPS_PROXY=''
$env:ALL_PROXY=''
```

14. Complete the versioned, user-readable release notes created by
    `release-version-prepare`.
   - `User-visible changes` must identify the salient behavior, workflow, output,
     compatibility, or hardware-support changes that a user can observe since
     `previousRelease`.
   - `Stability and reliability` must identify significant crash, corruption,
     recovery, target-selection, build-repeatability, validation, or release-safety
     improvements since `previousRelease`.
   - Be specific about effects rather than listing commit subjects. Do not
     fabricate a change to fill a section; if there was no significant change in
     a required category, state that explicitly and explain what remained
     unchanged.
   - `Release files` must name both hardware update HEX files and both matching
     release ZIPs.
   - `Full changelog` must link to the exact
     `previousRelease...currentRelease` GitHub comparison.
   - Do not publish GitHub's generated pull-request summary as the final body.
     Generated notes may be source material, but the checked versioned file is
     authoritative.

Validate the release-notes content:

```sh
just release-notes-check ../../release-evidence/release-checklist-vX.Y.Z.json
```

Record separate checklist evidence for `release-notes-user-visible` and
`release-notes-reliability`. Before integrating the candidate, run the combined
publication gate, which checks source state, notes content, and checklist
evidence:

```sh
just release-publication-check ../../release-evidence/release-checklist-vX.Y.Z.json candidate
```

15. After the candidate commit and checklist evidence are ready and pushed,
    create a clean integration worktree:

```sh
just release-integration-worktree X.Y.Z
```

This creates `/private/tmp/SignalSlinger-release-vX.Y.Z`, checks out `main`
there, and stages the repository's renormalized `Development2` merge without
committing. It intentionally leaves the original checkout and unrelated KiCad
edits untouched.

16. After explicit approval, review and commit the staged merge in the release
    worktree, push `main`, and re-run `just release-preflight` there from the
    clean integrated commit. Rebuild the packages, record their hashes, update
    the checklist `sourceCommit` to this exact main commit, and run:

```sh
just release-publication-check /absolute/path/to/release-checklist-vX.Y.Z.json release
```

17. Create and push the annotated tag, then create the GitHub release:
   - Use a prerelease or a normal release on `Development2` according to the requested release channel.
   - Use a normal release on `main` unless a prerelease is explicitly requested.
   - Always pass the checked file to `gh release create` with `--notes-file`.
     A release must not be published when `release-notes-check` fails.

For example, from this firmware directory:

```sh
gh release create vX.Y.Z \
  --repo OpenARDF/SignalSlinger \
  --title "SignalSlinger vX.Y.Z" \
  --notes-file release-notes/vX.Y.Z.md \
  <both-update-hex-files> <both-release-zip-files>
```

Use `--prerelease` when the approved checklist channel is `prerelease`.
18. Upload both hardware update HEX files and both matching release ZIPs.
19. Download the published assets into a fresh directory and independently
    verify their hashes, package validation, and tag target. Confirm the
    published body exactly matches the checked file:

```sh
just release-notes-remote-check /absolute/path/to/release-checklist-vX.Y.Z.json
```

20. Verify remotely:
   - the release page includes both `.hex` files
   - the release body exactly matches `release-notes/vX.Y.Z.md`
   - the notes retain substantive `User-visible changes` and
     `Stability and reliability` sections
   - `main` README points to stable downloads
   - `Development2` README points to the intended development-branch downloads

Update the release checklist through `remote-release-verified` and run:

```sh
just release-checklist /absolute/path/to/release-checklist-vX.Y.Z.json final
```

After the final phase passes, copy the completed checklist into the main
worktree, commit and push it as post-tag release evidence, and verify the
release tag still points at the clean package-build commit. Do not move the tag
to the later evidence commit.

## Notes

- `build-firmware.ps1` is the standard local build entry point for patch verification and can also be reused by other scripts.
- `verify-firmware-hashes.ps1` builds both hardware targets, reports SHA256 hashes for the copied `.hex` files, and restores the original active hardware target afterward.
- Use `-OutputDir` or `-KeepArtifacts` with `verify-firmware-hashes.ps1` if you want to keep the copied comparison artifacts.
- `prepare-release.ps1` remains the Windows path for older standalone HEX asset preparation. The Mac release package path reads the version directly from `defs.h` and checks matching README asset references.
- The release-prep flow should build `Release`, not `Debug`.
- Current update assets are named
  `SignalSlinger-Update-vX.Y.Z-HW-3.4.hex` and
  `SignalSlinger-Update-vX.Y.Z-HW-3.5.hex`. Each hardware target also has a
  matching `SignalSlinger-vX.Y.Z-HW-3.x-Release-Files.zip`.
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
