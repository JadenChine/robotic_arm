## How the BLE Interface Works (End-to-End Data Flow)

### 1. Advertising & Connection

This protocol is a **single-owner command stream** over BLE. Multiple centrals may be connected, but only one is considered the current “owner” at a time.

#### Firmware (Pico W, core 1)

`arm_ble_core1_entry()`:

1. Initialises the CYW43 Wi‑Fi/BT chip.
2. Sets up BTstack and the ATT server.
3. Starts advertising as:
   - **Local name:** `ArmCtrl`
   - **Service UUID:** `0xFFF0`

The firmware allows **up to 2 concurrent BLE connections**, tracked in `g_connections[]` (2 fixed slots). If a third central connects, the firmware will promptly disconnect it once it detects the slot limit.

#### GATT surface (what centrals see)

Custom service `0xFFF0` exposes three characteristics:

- `0xFFF1` — **Control Point** (write-with-response)
  - `0x01` = TAKE_CONTROL
  - `0x02` = RELEASE_CONTROL
- `0xFFF2` — **Owner Status** (read, dynamic)
  - Returns 1 byte: `0x01` if *this connection* is the owner; else `0x00`
- `0xFFF3` — **Joint Targets** (write / write-without-response)
  - Payload: 6 × `int16` (deci-degrees), little-endian → 12 bytes total

#### Mac app connection lifecycle

`BLEManager` on macOS:

1. Creates `CBCentralManager` and waits for `.poweredOn`.
2. Scans for peripherals advertising `0xFFF0`.
3. On Connect: connects, discovers service `0xFFF0`, then discovers `0xFFF1/0xFFF2/0xFFF3`.
4. Periodically reads `0xFFF2` to update whether it is currently the owner.

The UI is designed so you can connect, take/release control, and disconnect back to a clean scanning state.

### 2. Taking Ownership

Ownership is intentionally **not token-based**. There is no password, no shared key, and no monotonic token counter. The firmware simply records which **connection handle** currently owns.

At the ATT level, the Control Point command is "first byte only": the payload must be at least 1 byte, and only `buffer[0]` is interpreted (any additional bytes are ignored). Unknown command values are rejected.

#### TAKE_CONTROL (`0x01`) — last-writer wins

**Mac → Pico**

- User presses “Take Control”.
- App writes `0x01` to `0xFFF1` using **write-with-response**.

**Pico firmware behavior**

- The writer’s BLE connection handle becomes the owner (`g_owner_handle = connection_handle`).
- If another central was previously the owner, ownership transfers immediately.
- TAKE_CONTROL never triggers safe pose by itself.

**Mac app behavior**

After a successful TAKE_CONTROL write, the macOS app immediately sends one joint-target packet reflecting the current slider positions (so the arm updates right away even if you don't touch a slider again).

**Implication (important edge case)**

If Central A owned and was holding a non-zero pose, and Central B steals ownership but hasn’t sent any target packets yet, the arm will continue holding A’s last commanded pose until B sends new joint targets.

#### RELEASE_CONTROL (`0x02`) — owner-only + safe pose

**Mac → Pico**

- User presses “Release”.
- App writes `0x02` to `0xFFF1` using **write-with-response**.

**Pico firmware behavior**

- If the writer is **not** the current owner, the write fails with an ATT authorization error (release cannot be forced by a non-owner).
- If the writer **is** the owner, firmware clears `g_owner_handle` and enqueues a **safe pose** event (`ARM_BLE_EVENT_SAFE_POSE`).

Safe pose means: all joint targets become 0° and the servo loop is forced to treat outputs as dirty so the next control tick sends an update.

#### Owner Status (`0xFFF2`) — “am I the owner?”

`0xFFF2` is a dynamic read-only characteristic that returns exactly 1 byte:

- `0x01` if **the reading connection** is currently the owner
- `0x00` otherwise (including when there is no owner)

This characteristic is intentionally asymmetric:

- Two connected centrals can read `0xFFF2` and see different values at the same time.
- It does not identify who the owner is; it only answers “do I own?”.

**Mac behavior:** after TAKE_CONTROL or RELEASE_CONTROL completes, the app reads `0xFFF2` to refresh `hasControl`.

### 3. Sending Joint Targets (Slider → Servo)

**Mac → Pico:**
1. User drags a slider → `updateJointTargetsDegrees([Double])` marks `pendingJointTargetsDirty`.
2. A 60 Hz timer (`jointSendTimer`, ~0.0167 s) calls `flushPendingJointTargetsIfNeeded()`.
3. `encodeJointTargetsPacket()` converts each `Double` degree → `Int16` deci-degrees (×10), clamped to ±900, packed as 12 bytes of little-endian `int16`.
4. Written to `0xFFF3` (prefers write-without-response if supported). Writes are gated by CoreBluetooth backpressure when using write-without-response.

The Mac app continuously writes targets while sliders are moved, but only when its connection is currently the owner.

**Pico core 1 (BLE):** `att_write_callback()` → `handle_joint_targets_write()`:

- If writer ≠ owner: **ignore and return success** (no-op).
  - No queue event is generated.
  - No servo targets are changed.
  - Payload length is not validated for non-owners (even malformed packets are simply ignored).
- If writer == owner:
  - Reject if payload ≠ 12 bytes.
  - Decode 6 × `int16` LE → `arm_ble_event_t.joint_target_deci_deg[]`.
  - Enqueue `ARM_BLE_EVENT_SET_TARGETS` into the cross-core `queue_t`.

**Pico core 0 (main loop):** `process_ble_events()` dequeues the event → `apply_ble_joint_targets()`:
- Converts deci-degrees to millidegrees (`value * 100`)
- Writes into `g_joint_target_mdeg[0..5]`
- Leaves servo transmission to the control-tick scheduler

**Control tick (20 Hz timer ISR → main loop):** `send_joint_targets_if_dirty()`:
- For each joint: clamp → direction/offset transform → `mdeg_to_raw_position()` (maps ±120° → 0–1000 raw)
- Deadband filter against last-sent raw values
- Only dirty joints get batched into one `servo_bus_move_multiple()` call
- Sent over UART to the xArm controller board → physical servos

### 4. Ownership Timeout / Disconnect Safety

There is **no inactivity timeout**. Ownership does not expire based on time or target write rate.

Instead, safe behavior is guaranteed by explicit termination events:

1. **Release by the owner** (`RELEASE_CONTROL`): clears ownership and commands safe pose.
2. **Disconnect of the owner** (`HCI_EVENT_DISCONNECTION_COMPLETE`): clears ownership and commands safe pose.

Non-owner disconnects do not affect ownership.

#### Release vs Disconnect (app-level meaning)

From the Mac app’s perspective:

- **Release**: you remain connected, but you are no longer the owner; the arm is commanded to safe pose.
- **Disconnect**: the BLE link is torn down (you must reconnect and rediscover characteristics later). The app performs best-effort **Release → Disconnect** so the arm centers immediately even if the OS delays the disconnect.

---

## Assessment: What Looks Good

1. **Cross-core safety** — the `queue_t` is the only data path between core 1 (BLE) and core 0 (servo loop). No shared mutable state is accessed from both cores without the queue. This is correct.
2. **Ownership model** — single-owner handle semantics with immediate steal on `TAKE_CONTROL`, and a simple per-connection owner-status read (`0xFFF2`) that avoids fragile token matching.
3. **Queue backpressure** — `enqueue_event()` drops the oldest event if the queue is full, which is the right choice for a real-time target stream (latest value matters most).
4. **GATT definition** — `0xFFF3` has both `WRITE` and `WRITE_WITHOUT_RESPONSE`, allowing the Mac to use the faster path. The Mac code correctly prefers `.withoutResponse`.
5. **Advertisement** — correctly includes `0xFFF0` in the 16-bit service UUID list, so `scanForPeripherals(withServices: [armServiceUUID])` will find it.
6. **Packet encoding matches decoding** — Mac sends 6 × `Int16` LE (deci-degrees); firmware reads 6 × `int16` LE via `little_endian_read_16` and casts to `int16_t`. Bit-exact match.
7. **Non-owner behavior is gentle** — non-owner target writes are ACK’d but ignored, so a second controller can stream continuously without error spam while the arm remains stable.
8. **Release/disconnect safety is explicit** — safe pose is guaranteed on owner release and owner disconnect, without relying on an activity watchdog.
9. **BTstack config** — `MAX_NR_HCI_CONNECTIONS = 2` matches the firmware's 2-slot connection table.
10. **CMake** — correctly links `pico_btstack_ble`, `pico_btstack_cyw43`, `pico_cyw43_arch_none`, uses `pico_btstack_make_gatt_header`, and defaults `PICO_BOARD` to `pico_w`.

---

## Concerns and Issues

### Issue 4 — No `ENABLE_LE_SECURE_CONNECTIONS` / no pairing

The BLE link has no security. Anyone in radio range can connect, take control, and drive the arm. The `btstack_config.h` only sets `ENABLE_LE_PERIPHERAL` with `IO_CAPABILITY_NO_INPUT_NO_OUTPUT`. This is fine for development but should be hardened before any deployment near people.

### Issue 5 — `cyw43_arch_init()` is called on core 1

This is valid with `pico_cyw43_arch_none` (threadsafe-background is not linked, so no IRQ contention), but if you ever switch to `pico_cyw43_arch_threadsafe_background`, the CYW43 arch must be initialised on core 0. Document this constraint.

### Issue 6 — Heartbeat LED pin 16 is CYW43-managed on Pico W

`servo_platform_config.h` sets `SERVO_BUS_HEARTBEAT_LED_PIN` to 16. On a Pico W, GPIO 25 is the on-board LED but it's driven through the CYW43 chip (not via `gpio_put`). GPIO 16 is a normal GPIO pin, so this only works if you have an external LED on GP16. If you intend to use the on-board LED, you need `cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ...)` instead of `gpio_put(16, ...)`. The current code won't crash, but the on-board LED won't blink.

---

## How to Verify Everything Works

### Step 1: Build and flash
```bash
cd src/arm
cmake -S . -B build -DPICO_BOARD=pico_w
./scripts/build_and_flash.sh --connect
```

In the serial monitor you should see:
```
HiWonder Arm Joint Target Controller
  Backend       : xArm controller protocol
  ...
  BLE role      : peripheral (core1)
  BLE timeout   : 1500 ms to safe pose
initializing servo bus... ok
ble core launched
ble init ok on core1
ble advertising started (service 0xFFF0)
control timer started: 20 Hz
```

### Step 2: Scan and connect from the Mac
```bash
cd src/mac-app
swift run
```
- The app should auto-scan and show "ArmCtrl" in the peripheral list.
- Click "Connect". Status should go to "Connected and ready".

### Step 3: Take control
- Press "Take Control". The serial monitor should print `ble ownership granted to handle=X`.
- The Mac UI should show "Control: OWNER".

### Step 4: Move sliders
- Drag any joint slider. The serial monitor should show no errors.
- The physical servo for that joint should move. Verify each joint 1–6 individually.
- Verify that returning a slider to 0° centers that servo.

### Step 5: Test disconnect safety
- Take control, move a slider to a non-zero position.
- **Kill the Mac app** (Cmd+Q) without pressing Release.
- The serial monitor should print `ble ownership cleared: owner disconnected`.
- All servos should return to center (safe pose).

### Step 6: Test release
- Reconnect and take control. Move a slider. Press "Release".
- The arm should return to center (safe pose).

### Step 7: Verify ignored unauthorized writes
- Connect a second BLE client (e.g. nRF Connect on iPhone).
- Have the Mac take control. Try writing 12 bytes to `0xFFF3` from nRF Connect.
- The write should succeed at the ATT level, but the arm should not move (ignored because it's not the owner).

---

**Bottom line:** The implementation is solid and well-structured. The data pipeline from slider to servo is correctly encoded, queued across cores, and transmitted. The main items to address before real use are BLE link security (Issue 4) and awareness that the LED heartbeat won't toggle the on-board LED on Pico W (Issue 6).
