#!/usr/bin/env bash

set -euo pipefail

BAUD="${BAUD:-115200}"

sleep 1
echo "Searching for Pico USB device..."

shopt -s nullglob
devices=(/dev/cu.usbmodem* /dev/tty.usbmodem* /dev/ttyACM* /dev/ttyUSB*)
shopt -u nullglob

if [[ ${#devices[@]} -eq 0 ]]; then
    echo "Error: No Pico device found"
    echo "Make sure your Pico is connected via USB"
    exit 1
fi

DEVICE="${devices[0]}"
echo "Found device: ${DEVICE}"
echo "Opening picocom serial monitor..."
echo "Baud: ${BAUD}"
echo "Press Ctrl+A then Ctrl+X to exit"
echo

if ! command -v picocom >/dev/null 2>&1; then
    echo "Error: picocom is not installed"
    echo "Install it with: brew install picocom"
    exit 1
fi

# Keep terminal settings sane before launching picocom.
if [[ -e /dev/tty ]]; then
    stty sane </dev/tty 2>/dev/null || true
fi

# Picocom provides two-way serial I/O. Use CRLF and delete/backspace mappings for smoother typing.
exec picocom \
    --baud "${BAUD}" \
    --flow n \
    --parity n \
    --databits 8 \
    --stopbits 1 \
    --emap crcrlf,delbs \
    --omap crcrlf \
    "${DEVICE}"
