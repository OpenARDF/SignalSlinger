#!/bin/sh

set -eu

firmware_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
repo_root=$(git -C "$firmware_root" rev-parse --show-toplevel)
failure=0

expect_ignored() {
	path=$1
	if git -C "$repo_root" check-ignore -q --no-index "$path"; then
		printf '%s\n' "ignored as expected: $path"
	else
		printf '%s\n' "ERROR: expected ignored path is visible: $path" >&2
		failure=1
	fi
}

expect_visible() {
	path=$1
	if git -C "$repo_root" check-ignore -q --no-index "$path"; then
		printf '%s\n' "ERROR: project asset would be ignored: $path" >&2
		failure=1
	else
		printf '%s\n' "visible as expected: $path"
	fi
}

expect_eol() {
	expected=$1
	path=$2
	actual=$(git -C "$repo_root" check-attr eol -- "$path" | sed 's/^.*: eol: //')
	if [ "$actual" = "$expected" ]; then
		printf '%s\n' "eol=$expected: $path"
	else
		printf '%s\n' "ERROR: $path has eol=$actual, expected $expected" >&2
		failure=1
	fi
}

expect_ignored "Software/AVR128DA28/.vs/example.suo"
expect_ignored "Software/AVR128DA28/tmp/avr-tools/example"
expect_ignored "Software/AVR128DA28/release-packages/example"
expect_ignored "Software/AVR128DA28/SignalSlinger/Release/example.hex"
expect_ignored "KiCad/SignalSlinger/SignalSlinger-backups/example.zip"
expect_visible "Software/AVR128DA28/SignalSlinger/defs.h"
expect_visible "Software/AVR128DA28/release-notes/v9.9.9.md"
expect_visible "KiCad/SignalSlinger/SignalSlinger.kicad_pro"
expect_visible "release-evidence/release-checklist-v9.9.9.json"

expect_eol "lf" "README.md"
expect_eol "lf" "Software/AVR128DA28/Justfile"
expect_eol "lf" "Software/AVR128DA28/.gitleaksignore"
expect_eol "lf" "Software/AVR128DA28/release-notes/v9.9.9.md"
expect_eol "lf" "Software/AVR128DA28/scripts/package-avr-release.mjs"
expect_eol "lf" "Software/AVR128DA28/scripts/setup-avr-build-macos.sh"
expect_eol "crlf" "Software/AVR128DA28/build-release-package.ps1"
expect_eol "crlf" "Software/AVR128DA28/SignalSlinger/SignalSlinger.cppproj"

while IFS= read -r json_file; do
	if ! jq empty "$repo_root/$json_file"; then
		printf '%s\n' "ERROR: invalid JSON: $json_file" >&2
		failure=1
	fi
done <<EOF
$(git -C "$repo_root" ls-files '*.json')
EOF

gitleaks_ignore="$firmware_root/.gitleaksignore"
ignore_count=$(wc -l <"$gitleaks_ignore" | tr -d ' ')
if [ "$ignore_count" -ne 6 ] ||
	! rg -q \
		'^[0-9a-f]{40}:(KiCad/SignalSlinger|SignalSlinger/KiCad/SignalSlinger)/fp-info-cache:generic-api-key:[0-9]+$' \
		"$gitleaks_ignore" ||
	rg -q -v \
		'^[0-9a-f]{40}:(KiCad/SignalSlinger|SignalSlinger/KiCad/SignalSlinger)/fp-info-cache:generic-api-key:[0-9]+$' \
		"$gitleaks_ignore"; then
	printf '%s\n' "ERROR: .gitleaksignore must contain only the six reviewed KiCad cache fingerprints" >&2
	failure=1
else
	printf '%s\n' "exact gitleaks baseline: 6 reviewed KiCad cache fingerprints"
fi

if ! rg -q 'release_evidence_exclusion.*release-evidence' "$firmware_root/Justfile"; then
	printf '%s\n' "ERROR: normal staging must exclude release-evidence" >&2
	failure=1
fi

if ! rg -q '^release-notes-check checklist:' "$firmware_root/Justfile" ||
	! rg -q '^release-notes-current-check:' "$firmware_root/Justfile" ||
	! rg -q '^release-publication-check checklist phase:' "$firmware_root/Justfile" ||
	! rg -q '^release-notes-remote-check checklist:' "$firmware_root/Justfile"; then
	printf '%s\n' "ERROR: required release-note validation recipes are missing" >&2
	failure=1
fi

if ! rg -q \
	'^release-preflight:.*release-notes-current-check' \
	"$firmware_root/Justfile"; then
	printf '%s\n' "ERROR: release preflight must require checked release notes" >&2
	failure=1
fi

if ! rg -q '"id": "release-notes-user-visible"' "$firmware_root/release-checklist-template.json" ||
	! rg -q '"id": "release-notes-reliability"' "$firmware_root/release-checklist-template.json"; then
	printf '%s\n' "ERROR: release checklist must separately require user-visible and reliability notes" >&2
	failure=1
fi

if ! rg -q -- '--notes-file release-notes/vX\.Y\.Z\.md' "$firmware_root/RELEASE_WORKFLOW.md" ||
	! rg -q 'User-visible changes' "$firmware_root/RELEASE_WORKFLOW.md" ||
	! rg -q 'Stability and reliability' "$firmware_root/RELEASE_WORKFLOW.md"; then
	printf '%s\n' "ERROR: GitHub release-note publication policy is incomplete" >&2
	failure=1
fi

if rg -q 'SignalSlinger-vX\.Y-3\.[45]\.hex' "$firmware_root/RELEASE_WORKFLOW.md"; then
	printf '%s\n' "ERROR: legacy two-component release asset naming is documented" >&2
	failure=1
fi

git -C "$repo_root" diff --check
git -C "$repo_root" diff --cached --check

if [ "$failure" -ne 0 ]; then
	exit 1
fi

printf '%s\n' "PASS repository policy"
