# macOS BLE Arm Controller (SwiftUI)

This folder contains a native macOS controller app that talks to the arm Pico W over BLE.

## What It Does

- scans for BLE peripherals exposing service `0xFFF0`
- connects and discovers control characteristics:
  - `0xFFF1` control point (`TAKE_CONTROL=0x01`, `RELEASE_CONTROL=0x02`)
  - `0xFFF2` owner token (`uint32`, little-endian, read)
  - `0xFFF3` joint targets (6 x `int16` deci-degrees, little-endian)
- provides six sliders (`-90.0°` to `+90.0°`)
- sends slider packets at up to 20 Hz while dragging

## Run

From this directory:

```bash
swift run
```

If macOS asks for Bluetooth permissions, allow access.

## Notes

- Slider values are sent in deci-degrees (`0.1°` units).
- The firmware accepts target packets only from the current control owner.
- If ownership times out or owner disconnects, firmware switches to safe pose (all targets zero).
