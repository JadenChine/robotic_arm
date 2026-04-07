#!/usr/bin/env bash

set -euo pipefail

sleep 1
echo "Searching for Pico USB device..."

shopt -s nullglob
devices=(/dev/tty.usbmodem*)
shopt -u nullglob

if [[ ${#devices[@]} -eq 0 ]]; then
    echo "Error: No Pico device found"
    echo "Make sure your Pico is connected via USB"
    exit 1
fi

DEVICE="${devices[0]}"
echo "Found device: ${DEVICE}"
echo "Connecting to Pico serial monitor..."
echo "Press Ctrl+C to exit"
echo

if command -v pipenv >/dev/null 2>&1; then
    PYTHON_CMD=(pipenv run python)
else
    echo "Warning: pipenv not found"
fi

DEVICE="${DEVICE}" "${PYTHON_CMD[@]}" - <<'PY'
import os
import sys

try:
    import serial
except ImportError:
    print("Error: pyserial is required for serial_connect.sh")
    print("Install it with: pipenv install pyserial")
    sys.exit(1)


device = os.environ["DEVICE"]

try:
    ser = serial.Serial(device, 115200, timeout=0.1)
    print("Connected! Reading output...\n", flush=True)
    while True:
        waiting = ser.in_waiting
        if waiting:
            data = ser.read(waiting)
            sys.stdout.buffer.write(data)
            sys.stdout.flush()
except KeyboardInterrupt:
    print("\n\nDisconnected")
    sys.exit(0)
except Exception as exc:
    print(f"\nError: {exc}")
    sys.exit(1)
PY
