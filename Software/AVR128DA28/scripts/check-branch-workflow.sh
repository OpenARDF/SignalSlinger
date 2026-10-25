#!/usr/bin/env bash

set -euo pipefail

firmware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(git -C "$firmware_root" rev-parse --show-toplevel)"
collector="${1:-}"
mode="${2:-}"

if [[ "$collector" != "Development1" && "$collector" != "Development1-KiCad" ]]; then
	printf 'ERROR: collector must be Development1 or Development1-KiCad\n' >&2
	exit 1
fi
if [[ -n "$mode" && "$mode" != "--promotion" && "$mode" != "--sync" ]]; then
	printf 'ERROR: unsupported mode: %s\n' "$mode" >&2
	exit 1
fi

current_branch="$(git -C "$repo_root" branch --show-current)"
if [[ -z "$current_branch" ]]; then
	printf 'ERROR: branch routing cannot be checked from detached HEAD\n' >&2
	exit 1
fi

main_ref="origin/main"
collector_ref="origin/$collector"

# The Justfile fetches before invoking this script. Always validate against the
# fetched remote tips so stale local branches cannot pass a routing gate.
for required_ref in "$main_ref" "$collector_ref"; do
	if git -C "$repo_root" show-ref --verify --quiet "refs/remotes/$required_ref"; then
		continue
	fi
	printf 'ERROR: fetched remote branch is unavailable locally: %s\n' "$required_ref" >&2
	exit 1
done

if [[ "$mode" == "--sync" ]]; then
	if [[ "$current_branch" == "$collector" || "$current_branch" == "main" ]]; then
		printf 'ERROR: synchronization must run on a focused topic branch, not %s\n' \
			"$current_branch" >&2
		exit 1
	fi

	collector_oid="$(git -C "$repo_root" rev-parse "$collector_ref")"
	main_oid="$(git -C "$repo_root" rev-parse "$main_ref")"
	parents="$(git -C "$repo_root" rev-list --parents -n 1 HEAD)"
	IFS=' ' read -r _ first_parent second_parent extra_parent <<EOF
$parents
EOF
	# Exact parents make synchronization auditable: no extra topic commits,
	# rebases, or squash-equivalent content can masquerade as baseline ancestry.
	if [[ -n "${extra_parent:-}" || "${first_parent:-}" != "$collector_oid" ||
		"${second_parent:-}" != "$main_oid" ]]; then
		printf 'ERROR: sync HEAD must merge %s first and %s second\n' \
			"$collector_ref" "$main_ref" >&2
		exit 1
	fi
	if ! git -C "$repo_root" diff --quiet || ! git -C "$repo_root" diff --cached --quiet; then
		printf 'ERROR: synchronization checkout must be clean\n' >&2
		exit 1
	fi

	printf 'PASS branch route: %s -> %s (sync)\n' "$current_branch" "$collector"
	exit 0
fi

if ! git -C "$repo_root" merge-base --is-ancestor "$main_ref" "$collector_ref"; then
	printf 'ERROR: %s does not contain the fetched main baseline\n' "$collector_ref" >&2
	exit 1
fi

if [[ "$mode" == "--promotion" ]]; then
	if [[ "$current_branch" != "$collector" ]]; then
		printf 'ERROR: promotion preflight must run on %s, not %s\n' \
			"$collector" "$current_branch" >&2
		exit 1
	fi
	if [[ "$(git -C "$repo_root" rev-parse HEAD)" != "$(git -C "$repo_root" rev-parse "$collector_ref")" ]]; then
		printf 'ERROR: local %s does not match %s\n' "$collector" "$collector_ref" >&2
		exit 1
	fi
	diff_base="$main_ref"
else
	if [[ "$current_branch" == "$collector" ]]; then
		if [[ "$(git -C "$repo_root" rev-parse HEAD)" != "$(git -C "$repo_root" rev-parse "$collector_ref")" ]]; then
			printf 'ERROR: local %s does not match %s\n' "$collector" "$collector_ref" >&2
			exit 1
		fi
	elif ! git -C "$repo_root" merge-base --is-ancestor "$collector_ref" HEAD; then
		printf 'ERROR: %s is not based on %s\n' "$current_branch" "$collector_ref" >&2
		exit 1
	fi
	diff_base="$collector_ref"
fi

failure=0
while IFS= read -r path; do
	[[ -n "$path" ]] || continue
	if [[ "$collector" == "Development1" ]]; then
		if [[ "$path" == KiCad/* ]]; then
			printf 'ERROR: firmware/software collector contains KiCad path: %s\n' "$path" >&2
			failure=1
		fi
	else
		case "$path" in
		KiCad/* | README.md | \
			Software/AVR128DA28/Justfile | \
			Software/AVR128DA28/CODEX_WORKFLOW.md | \
			Software/AVR128DA28/RELEASE_WORKFLOW.md | \
			Software/AVR128DA28/scripts/check-branch-workflow.sh | \
			Software/AVR128DA28/scripts/test-branch-workflow.sh | \
			Software/AVR128DA28/scripts/check-kicad.sh | \
			Software/AVR128DA28/scripts/check-spice.sh | \
			Software/AVR128DA28/scripts/kicad-fontconfig.conf | \
			Software/AVR128DA28/scripts/check-repository-policy.sh) ;;
		*)
			printf 'ERROR: KiCad collector path is outside KiCad or the narrow workflow bootstrap allowlist: %s\n' \
				"$path" >&2
			failure=1
			;;
		esac
	fi
done < <(git -C "$repo_root" diff --name-only "$diff_base"...HEAD)

if [[ "$failure" -ne 0 ]]; then
	exit 1
fi

printf 'PASS branch route: %s -> %s%s\n' "$current_branch" "$collector" \
	"${mode:+ (${mode#--})}"
