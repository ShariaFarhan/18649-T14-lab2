#!/usr/bin/env bash
set -euo pipefail

port="${1:-}"

if [[ -z "$port" ]]; then
	for candidate in /dev/cu.usbmodem* /dev/cu.usbserial*; do
		if [[ -e "$candidate" ]]; then
			port="$candidate"
			break
		fi
	done
fi

if [[ -z "$port" ]]; then
	echo "No Nucleo serial port found." >&2
	echo "Connect the board, then retry or pass a port explicitly:" >&2
	echo "  $0 /dev/cu.usbmodemXXXX" >&2
	exit 1
fi

echo "Opening $port at 115200 baud."
echo "Press Ctrl-A then Ctrl-X to exit picocom."
exec picocom \
	--baud 115200 \
	--flow none \
	--omap crcrlf \
	--raise-dtr \
	--raise-rts \
	"$port"
