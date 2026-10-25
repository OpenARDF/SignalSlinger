#!/bin/sh

set -eu

firmware_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
repo_root=$(git -C "$firmware_root" rev-parse --show-toplevel)

deck_list=$(find "$repo_root/KiCad" -type f \( -name '*.cir' -o -name '*.sp' -o -name '*.spice' \) -print)
if [ -z "$deck_list" ]; then
	printf '%s\n' "PASS SPICE: no simulation decks are present"
	exit 0
fi

if ! command -v ngspice >/dev/null 2>&1; then
	printf '%s\n' "ERROR: ngspice is required when simulation decks are present" >&2
	exit 1
fi

tmp_root=${TMPDIR:-/tmp}
work_dir=$(mktemp -d "$tmp_root/signalslinger-spice.XXXXXX")
trap 'rm -rf "$work_dir"' EXIT HUP INT TERM

failure=0
while IFS= read -r deck; do
	[ -n "$deck" ] || continue
	log="$work_dir/$(basename "$deck").log"
	if ! ngspice -b -o "$log" "$deck"; then
		printf '%s\n' "ERROR: SPICE check failed: ${deck#"$repo_root/"}" >&2
		cat "$log" >&2
		failure=1
	elif rg -i -q '(^|[^a-z])(warning|error|fatal)([^a-z]|$)' "$log"; then
		printf '%s\n' "ERROR: SPICE emitted an unwaived diagnostic: ${deck#"$repo_root/"}" >&2
		cat "$log" >&2
		failure=1
	else
		printf '%s\n' "PASS SPICE: ${deck#"$repo_root/"}"
	fi
done <<EOF
$deck_list
EOF

exit "$failure"
