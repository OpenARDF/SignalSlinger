#!/bin/sh

set -eu

firmware_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
repo_root=$(git -C "$firmware_root" rev-parse --show-toplevel)
missing=0

printf '%s\n' "Repository: $repo_root"
printf '%s\n' "Firmware: $firmware_root"
printf '%s\n' "Branch: $(git -C "$repo_root" branch --show-current)"

for tool in git rg jq node just gitleaks shellcheck shfmt zip unzip pwsh pymcuprog; do
	if command -v "$tool" >/dev/null 2>&1; then
		printf '%-12s %s\n' "$tool" "available"
	else
		printf '%-12s %s\n' "$tool" "MISSING"
		missing=1
	fi
done

if [ "$missing" -ne 0 ]; then
	exit 1
fi

cd "$firmware_root"
node ./scripts/build-avr-release.mjs --doctor
