from machine import Pin
from time import sleep
import LCD
import struct
import bluetooth
from ble_advertising import advertising_payload
from ble_simple_peripheral import BLESimplePeripheral

# ── State and operating mode setup ──────────────────────────────────────────────
# These variables hold the current values of the six encoder channels, the inverse-mode
# values, and flags to tell the display when a channel needs to be updated.
flag        = 0
mem         = 0
lcdFlag     = [False] * 6
invLcdFlag  = [False] * 3
modeFlag    = True
values      = [0] * 6
invValues   = [20] * 3
prev_states = [0] * 6

# BLE radio object and the peripheral wrapper used to send packets to a connected device.
ble = bluetooth.BLE()
Bluetooth = BLESimplePeripheral(ble)

# Stores learned teaching sequences for the TEACH mode.
teachValues = []

# Mode IDs and labels for the control system.
# FORWARD = normal forward-mode encoder values.
# INVERSE = inverse / mapped input values.
# TEACH = stores and replays preset values.
FORWARD, INVERSE, TEACH = range(3)
MODES = [FORWARD, INVERSE, TEACH]
NAMES = ["FORWARD", "INVERSE", " TEACH "]
mode  = FORWARD

# Quadrature encoder transition table.
# Each entry maps the previous state and current state to a direction change:
# 0, -1, +1, 0 etc. This is the classic 4x resolution lookup used for encoders.
TRANSITIONS = [
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
]

# ── LCD setup ───────────────────────────────────────────────────────────────────
# The display is configured with a white foreground and black background.
White = LCD.RGB(250, 250, 250)
Black = LCD.RGB(0, 0, 0)
LCD.Init()
LCD.Clear(Black)

# Draw labels for each encoder and the mode area on the screen.
for i, label in enumerate(["Dial 1","Dial 2","Dial 3","Dial 4","Dial 5","Dial 6"]):
    LCD.Text(label, 10 + 56 * i, 280, White, Black)
LCD.Text("Mode", 366, 280, White, Black)

# ── Pin definitions ────────────────────────────────────────────────────────────
# Each encoder has two phase inputs (A and B). The button pins are used for mode changes
# and teach/save actions.
A1 = Pin(27, Pin.IN, Pin.PULL_DOWN)
B1 = Pin(28, Pin.IN, Pin.PULL_DOWN)
A2 = Pin(22, Pin.IN, Pin.PULL_DOWN)
B2 = Pin(26, Pin.IN, Pin.PULL_DOWN)
A3 = Pin(20, Pin.IN, Pin.PULL_DOWN)
B3 = Pin(21, Pin.IN, Pin.PULL_DOWN)
A4 = Pin(18, Pin.IN, Pin.PULL_DOWN)
B4 = Pin(19, Pin.IN, Pin.PULL_DOWN)
A5 = Pin(16, Pin.IN, Pin.PULL_DOWN)
B5 = Pin(17, Pin.IN, Pin.PULL_DOWN)
A6 = Pin(14, Pin.IN, Pin.PULL_DOWN)
B6 = Pin(15, Pin.IN, Pin.PULL_DOWN)

button1 = Pin(8, Pin.IN, Pin.PULL_DOWN)
button2 = Pin(9, Pin.IN, Pin.PULL_DOWN)
button3 = Pin(10, Pin.IN, Pin.PULL_DOWN)
button4 = Pin(11, Pin.IN, Pin.PULL_DOWN)

# Capture the starting encoder state so the script can calculate direction changes from
# the previous state on each interrupt.
prev_states[0] = (A1.value() << 1) | B1.value()
prev_states[1] = (A2.value() << 1) | B2.value()
prev_states[2] = (A3.value() << 1) | B3.value()
prev_states[3] = (A4.value() << 1) | B4.value()
prev_states[4] = (A5.value() << 1) | B5.value()
prev_states[5] = (A6.value() << 1) | B6.value()

# ── Encoder update (called by the interrupt handlers) ──────────────────────────
# This function reads the current A/B state, decides which direction the encoder moved,
# updates the relevant value, and limits it within a min/max range.
def update_encoder(idx):
    global values, prev_states, lcdFlag, invValues

    # Read the current phase state for the selected encoder.
    if idx == 0:
        current = (A1.value() << 1) | B1.value()
    elif idx == 1:
        current = (A2.value() << 1) | B2.value()
    elif idx == 2:
        current = (A3.value() << 1) | B3.value()
    elif idx == 3:
        current = (A4.value() << 1) | B4.value()
    elif idx == 4:
        current = (A5.value() << 1) | B5.value()
    elif idx == 5:
        current = (A6.value() << 1) | B6.value()

    # Combine previous and current states to index into the transition table.
    transition = (prev_states[idx] << 2) | current

    # In FORWARD mode, the six encoder values are updated directly.
    if mode == FORWARD:
        values[idx] = (values[idx] + TRANSITIONS[transition])
        if values[idx] > 180:
            values[idx] = 180
        elif values[idx] < -180:
            values[idx] = -180
        lcdFlag[idx] = True

    # In INVERSE or TEACH mode, the inverse values are changed instead.
    elif mode == INVERSE or mode == TEACH:
        invValues[idx] = (invValues[idx] + TRANSITIONS[transition])
        if invValues[idx] > 100:
            invValues[idx] = 100
        elif invValues[idx] < 20 and TRANSITIONS[transition] == -1 and invValues[idx] > -20:
            invValues[idx] = -20
        elif invValues[idx] > -20 and TRANSITIONS[transition] == 1 and invValues[idx] < 20:
            invValues[idx] = 20
        elif invValues[idx] < -100:
            invValues[idx] = -100
        invLcdFlag[idx] = True

    # Store the latest state so the next interrupt uses the right previous value.
    prev_states[idx] = current


# Creates an interrupt handler for each encoder channel.
def make_handler(idx):
    def handler(pin):
        update_encoder(idx)
    return handler

# ── Button and mode handling ─────────────────────────────────────────────────────
# This switch function is not used by the current UI but appears to be a memory toggle
# that saves and restores a prior value.
def switch(pin):
    global flag, values, mem
    if flag:
        values[0] = mem
        flag = 0
    else:
        mem = values[0]
        flag = 1

# Move to the next or previous operating mode.
# The values are reset when mode changes so display state matches the new mode.
def modeNext(pin):
    global mode, modeFlag, values
    mode = (mode + 1) % len(MODES)
    modeFlag = True
    values = [0] * 6
    invValues = [20] * 3


def modePrev(pin):
    global mode, modeFlag, values
    mode = (mode - 1) % len(MODES)
    modeFlag = True
    values = [0] * 6
    invValues = [20] * 3

# Sends the current control data over BLE as a packed byte sequence.
# The script packs 10 signed 8-bit values, which include the six forward values,
# the three inverse values, and the current mode.
def transmitData():
    data = values + invValues
    data.append(mode)
    for i in range(10):
        LCD.Number((data[i]),4,0, 10 + 40 * i, 100, White, Black)
    packed = struct.pack(f'{len(data)}b', *data)
    Bluetooth.send(packed)

# Saves the current control state into a teach list for later replay.
def saveTeach(pin):
    global teachValues
    if mode == TEACH:
        if len(teachValues) <= 5:
            data = values + invValues
            data.append(mode)
            teachValues.append(data)

# Replays all saved taught positions one by one over BLE.
def sendTeach(pin):
    if mode == TEACH and len(teachValues) > 0:
        for i, teach in enumerate(teachValues):
            for j in range(10):
                LCD.Number((teach[j]),4,0, 10 + 40 * j, 100 + 20 * i, White, Black)
            packed = struct.pack(f'{len(teach)}b', *teach)
            Bluetooth.send(packed)

# ── Attach interrupts for all encoders and buttons ──────────────────────────────
# Any change on the A/B lines triggers the encoder handler, which updates position.
TRIGGER = Pin.IRQ_RISING | Pin.IRQ_FALLING

h1 = make_handler(0)
A1.irq(trigger=TRIGGER, handler=h1)
B1.irq(trigger=TRIGGER, handler=h1)

h2 = make_handler(1)
A2.irq(trigger=TRIGGER, handler=h2)
B2.irq(trigger=TRIGGER, handler=h2)

h3 = make_handler(2)
A3.irq(trigger=TRIGGER, handler=h3)
B3.irq(trigger=TRIGGER, handler=h3)

h4 = make_handler(3)
A4.irq(trigger=TRIGGER, handler=h4)
B4.irq(trigger=TRIGGER, handler=h4)

h5 = make_handler(4)
A5.irq(trigger=TRIGGER, handler=h5)
B5.irq(trigger=TRIGGER, handler=h5)

h6 = make_handler(5)
A6.irq(trigger=TRIGGER, handler=h6)
B6.irq(trigger=TRIGGER, handler=h6)

# The physical buttons are wired to change modes or save/send teach points.
button1.irq(trigger=Pin.IRQ_RISING, handler=modeNext)
button2.irq(trigger=Pin.IRQ_RISING, handler=modePrev)
button3.irq(trigger=Pin.IRQ_RISING, handler=saveTeach)
button4.irq(trigger=Pin.IRQ_RISING, handler=sendTeach)

# ── Main loop ──────────────────────────────────────────────────────────────────
# This loop keeps the display updated and periodically transmits the current state.
while True:
    if 1:  # Bluetooth.is_connected() - currently left as a permanent condition.
        if mode == FORWARD:
            # Only redraw the encoder values that changed.
            if any(lcdFlag):
                for i in range(6):
                    if lcdFlag[i]:
                        LCD.Number(values[i], 4, 0, 18 + 56 * i, 304, White, Black)
                        lcdFlag[i] = False
        elif mode == INVERSE or mode == TEACH:
            # Redraw only the inverse values that changed.
            if any(invLcdFlag):
                for i in range(3):
                    if invLcdFlag[i]:
                        LCD.Number(invValues[i], 4, 0, 18 + 56 * i, 10, White, Black)
                        invLcdFlag[i] = False

        # When the mode changes, refresh the whole display once and print the mode label.
        if modeFlag:
            LCD.Text(NAMES[mode], 354, 304, White, Black)
            modeFlag = False
            for i in range(6):
                LCD.Number(values[i], 4, 0, 18 + 56 * i, 304, White, Black)
            for i in range(3):
                LCD.Number(invValues[i], 4, 0, 18 + 56 * i, 10, White, Black)

        # In non-teach modes, continuously transmit the current data to the robot/control system.
        if mode != TEACH:
            transmitData()

    sleep(0.001)