#!/bin/sh

set -eu

usage() {
	printf '%s\n' "usage: $0 <Development1|Development1-KiCad> [--promotion]" >&2
	exit 2
}

collector=${1:-}
mode=${2:-}

case "$collector" in
Development1 | Development1-KiCad) ;;
*) usage ;;
esac

case "$mode" in
'' | --promotion) ;;
*) usage ;;
esac

git rev-parse --show-toplevel >/dev/null
current_branch=$(git branch --show-current)

if [ -z "$current_branch" ]; then
	printf '%s\n' "ERROR: branch routing cannot be checked from detached HEAD" >&2
	exit 1
fi

resolve_ref() {
	ref=$1
	if git show-ref --verify --quiet "refs/heads/$ref"; then
		printf '%s\n' "$ref"
	elif git show-ref --verify --quiet "refs/remotes/origin/$ref"; then
		printf '%s\n' "origin/$ref"
	else
		printf '%s\n' "ERROR: required branch '$ref' was not found locally or on origin" >&2
		exit 1
	fi
}

collector_ref=$(resolve_ref "$collector")

if [ "$mode" = "--promotion" ]; then
	if [ "$current_branch" != "$collector" ]; then
		printf '%s\n' "ERROR: promotion preflight for $collector must run on $collector, not $current_branch" >&2
		exit 1
	fi

	main_ref=$(resolve_ref main)
	if ! git merge-base --is-ancestor "$main_ref" HEAD; then
		printf '%s\n' "ERROR: $collector must contain the current $main_ref baseline before promotion" >&2
		exit 1
	fi
	diff_base=$main_ref
else
	if [ "$current_branch" != "$collector" ] &&
		! git merge-base --is-ancestor "$collector_ref" HEAD; then
		printf '%s\n' "ERROR: topic branch $current_branch must start from $collector_ref" >&2
		exit 1
	fi
	diff_base=$collector_ref
fi

failure=0
changed_paths=$(git diff --name-only "$diff_base...HEAD")

if [ "$collector" = "Development1" ]; then
	while IFS= read -r path; do
		[ -n "$path" ] || continue
		case "$path" in
		KiCad/*)
			printf '%s\n' "ERROR: firmware/software collector cannot include KiCad path: $path" >&2
			failure=1
			;;
		esac
	done <<EOF
$changed_paths
EOF
else
	while IFS= read -r path; do
		[ -n "$path" ] || continue
		case "$path" in
		KiCad/* | README.md | \
			Software/AVR128DA28/Justfile | \
			Software/AVR128DA28/CODEX_WORKFLOW.md | \
			Software/AVR128DA28/RELEASE_WORKFLOW.md | \
			Software/AVR128DA28/scripts/check-branch-workflow.sh | \
			Software/AVR128DA28/scripts/check-kicad.sh | \
			Software/AVR128DA28/scripts/check-spice.sh | \
			Software/AVR128DA28/scripts/kicad-fontconfig.conf | \
			Software/AVR128DA28/scripts/check-repository-policy.sh)
			;;
		*)
			printf '%s\n' "ERROR: KiCad collector path is outside KiCad or the narrow workflow bootstrap allowlist: $path" >&2
			failure=1
			;;
		esac
	done <<EOF
$changed_paths
EOF
fi

if [ "$failure" -ne 0 ]; then
	exit 1
fi

if [ "$mode" = "--promotion" ]; then
	printf '%s\n' "PASS $collector promotion route: $diff_base...HEAD"
else
	printf '%s\n' "PASS $current_branch routes through $collector_ref"
fi
