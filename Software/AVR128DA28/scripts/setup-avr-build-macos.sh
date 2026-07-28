#!/bin/sh

set -eu

if [ "$(uname -s)" != "Darwin" ]; then
	printf '%s\n' "This setup helper is for macOS. Set AVR_TOOLCHAIN_ROOT and AVR_DFP_ROOT on other hosts." >&2
	exit 2
fi

script_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
workspace_root=$(CDPATH='' cd -- "$script_dir/.." && pwd)
tools_root="$workspace_root/tmp/avr-tools"
toolchain_root="$tools_root/avr8-gnu-toolchain-darwin_x86_64"
dfp_root="$tools_root/AVR-Dx_DFP/1.10.114"
download_dir=$(mktemp -d "${TMPDIR:-/tmp}/signalslinger-avr-setup.XXXXXX")

toolchain_url="https://ww1.microchip.com/downloads/aemDocuments/documents/DEV/ProductDocuments/SoftwareTools/avr8-gnu-toolchain-osx-3.7.0.518-darwin.any.x86_64.tar.gz"
toolchain_sha256="378b210cc82dc06599b5a45dede0ed188a9e87f2de045f12cf3547554edd6ec8"
dfp_url="http://packs.download.atmel.com/Atmel.AVR-Dx_DFP.1.10.114.atpack"
dfp_sha256="8eee80fe5cd01c54edd1959fd091043156156b4c05f84c7e5ea6e590714096fe"

cleanup() {
	rm -rf -- "$download_dir"
}
trap cleanup EXIT HUP INT TERM

verify_sha256() {
	path=$1
	expected=$2
	actual=$(shasum -a 256 "$path" | cut -d ' ' -f 1)
	if [ "$actual" != "$expected" ]; then
		printf 'SHA-256 mismatch for %s\nExpected: %s\nActual:   %s\n' "$path" "$expected" "$actual" >&2
		exit 2
	fi
}

install_toolchain() {
	if [ -x "$toolchain_root/bin/avr-g++" ]; then
		version=$("$toolchain_root/bin/avr-g++" -dumpversion)
		if [ "$version" = "7.3.0" ]; then
			printf 'AVR-GCC 7.3.0 already installed: %s\n' "$toolchain_root"
			return
		fi
		printf 'Unexpected compiler version in existing directory: %s\n' "$version" >&2
		exit 2
	fi
	if [ -e "$toolchain_root" ]; then
		printf 'Refusing to replace incomplete toolchain directory: %s\n' "$toolchain_root" >&2
		exit 2
	fi

	archive="$download_dir/avr8-gnu-toolchain.tar.gz"
	printf '%s\n' "Downloading the official AVR-GCC 7.3.0 macOS archive..."
	curl -fL --retry 2 -o "$archive" "$toolchain_url"
	verify_sha256 "$archive" "$toolchain_sha256"
	mkdir -p "$tools_root"
	tar -xzf "$archive" -C "$tools_root"
	if [ ! -x "$toolchain_root/bin/avr-g++" ]; then
		printf 'Compiler archive did not create the expected directory: %s\n' "$toolchain_root" >&2
		exit 2
	fi
}

install_dfp() {
	if [ -f "$dfp_root/Atmel.AVR-Dx_DFP.pdsc" ] &&
		[ -f "$dfp_root/include/avr/ioavr128da28.h" ] &&
		[ -d "$dfp_root/gcc/dev/avr128da28" ]; then
		printf 'AVR-Dx_DFP 1.10.114 already installed: %s\n' "$dfp_root"
		return
	fi
	if [ -e "$dfp_root" ]; then
		printf 'Refusing to replace incomplete device-pack directory: %s\n' "$dfp_root" >&2
		exit 2
	fi

	archive="$download_dir/Atmel.AVR-Dx_DFP.1.10.114.atpack"
	printf '%s\n' "Downloading the official Atmel AVR-Dx_DFP 1.10.114 archive..."
	curl -fL --retry 2 -o "$archive" "$dfp_url"
	verify_sha256 "$archive" "$dfp_sha256"
	mkdir -p "$dfp_root"
	unzip -q "$archive" -d "$dfp_root"
}

install_toolchain
install_dfp
node "$script_dir/build-avr-release.mjs" --doctor
