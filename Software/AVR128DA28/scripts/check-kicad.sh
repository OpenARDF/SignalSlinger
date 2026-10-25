#!/bin/sh

set -eu

firmware_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
repo_root=$(git -C "$firmware_root" rev-parse --show-toplevel)
project_root="$repo_root/KiCad/SignalSlinger"
schematic=SignalSlinger.kicad_sch
board=SignalSlinger.kicad_pcb
expected_drc_sha256=d7d1d6402bb9d74888b9200489501d575b2098d69caa2997805e6622305bc9ab

if [ -n "${KICAD_CLI:-}" ]; then
	kicad_cli=$KICAD_CLI
elif command -v kicad-cli >/dev/null 2>&1; then
	kicad_cli=$(command -v kicad-cli)
elif [ -x /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli ]; then
	kicad_cli=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
else
	printf '%s\n' "ERROR: kicad-cli is required" >&2
	exit 1
fi

tmp_root=${TMPDIR:-/tmp}
work_dir=$(mktemp -d "$tmp_root/signalslinger-kicad.XXXXXX")
trap 'rm -rf "$work_dir"' EXIT HUP INT TERM

if [ "$(uname -s)" = Darwin ]; then
	export FONTCONFIG_FILE="$firmware_root/scripts/kicad-fontconfig.conf"
	export XDG_CACHE_HOME="$work_dir/font-cache"
	mkdir -p "$XDG_CACHE_HOME"
fi

run_kicad() {
	label=$1
	shift
	stdout_file="$work_dir/$label.stdout"
	stderr_file="$work_dir/$label.stderr"
	status=0
	(
		cd "$project_root"
		"$kicad_cli" "$@"
	) >"$stdout_file" 2>"$stderr_file" || status=$?
	if [ -s "$stderr_file" ]; then
		printf '%s\n' "ERROR: $label emitted unwaived diagnostics:" >&2
		cat "$stderr_file" >&2
		exit 1
	fi
	cat "$stdout_file"
	return "$status"
}

erc_report="$work_dir/erc.rpt"
if ! run_kicad erc sch erc --exit-code-violations --severity-all -o "$erc_report" "$schematic"; then
	printf '%s\n' "ERROR: KiCad ERC found a violation" >&2
	cat "$erc_report" >&2
	exit 1
fi
if ! rg -q '^ \*\* ERC messages: 0  Errors 0  Warnings 0$' "$erc_report"; then
	printf '%s\n' "ERROR: KiCad ERC report did not confirm zero errors and warnings" >&2
	cat "$erc_report" >&2
	exit 1
fi

drc_report="$work_dir/drc.rpt"
drc_status=0
run_kicad drc pcb drc --exit-code-violations --severity-all -o "$drc_report" "$board" || drc_status=$?
if [ "$drc_status" -ne 5 ]; then
	printf '%s\n' "ERROR: KiCad DRC returned $drc_status; expected 5 for the reviewed inherited baseline" >&2
	exit 1
fi

normalized_report="$work_dir/drc.normalized.rpt"
sed -E 's/^\*\* Created on .* \*\*$/** Created on NORMALIZED **/' "$drc_report" >"$normalized_report"
if command -v shasum >/dev/null 2>&1; then
	actual_drc_sha256=$(shasum -a 256 "$normalized_report" | awk '{print $1}')
elif command -v sha256sum >/dev/null 2>&1; then
	actual_drc_sha256=$(sha256sum "$normalized_report" | awk '{print $1}')
else
	printf '%s\n' "ERROR: shasum or sha256sum is required" >&2
	exit 1
fi

if [ "$actual_drc_sha256" != "$expected_drc_sha256" ]; then
	printf '%s\n' "ERROR: KiCad DRC changed from the reviewed inherited baseline" >&2
	printf '%s\n' "expected sha256: $expected_drc_sha256" >&2
	printf '%s\n' "actual sha256:   $actual_drc_sha256" >&2
	cat "$drc_report" >&2
	exit 1
fi

printf '%s\n' "PASS KiCad ERC: 0 errors, 0 warnings"
printf '%s\n' "PASS KiCad DRC baseline: 75 inherited findings (11 errors, 64 warnings), no unreviewed change"
