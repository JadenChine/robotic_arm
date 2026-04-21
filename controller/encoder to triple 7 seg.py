from machine import Pin
from time import sleep_us
import LCD

# ── State ──────────────────────────────────────────────────────────────────────
flag        = 0
mem         = 0
lcdFlag     = [True] * 6
modeFlag    = True
values      = [0] * 6
prev_states = [0] * 6

FORWARD, INVERSE, TEACH = range(3)
MODES = [FORWARD, INVERSE, TEACH]
NAMES = ["FORWARD", "INVERSE", " TEACH "]
mode  = FORWARD

# 4x resolution quadrature transition table
TRANSITIONS = [
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
]

# ── LCD init ───────────────────────────────────────────────────────────────────
White = LCD.RGB(250, 250, 250)
Black = LCD.RGB(0, 0, 0)
LCD.Init()
LCD.Clear(Black)
for i, label in enumerate(["Dial 1","Dial 2","Dial 3","Dial 4","Dial 5","Dial 6"]):
    LCD.Text(label, 10 + 56 * i, 280, White, Black)
LCD.Text("Mode", 366, 280, White, Black)

# ── Pin definitions ────────────────────────────────────────────────────────────
A1 = Pin(21, Pin.IN, Pin.PULL_DOWN)
B1 = Pin(20, Pin.IN, Pin.PULL_DOWN)
A2 = Pin(19, Pin.IN, Pin.PULL_DOWN)
B2 = Pin(18, Pin.IN, Pin.PULL_DOWN)
# A3 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# B3 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# A4 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# B4 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# A5 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# B5 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# A6 = Pin(??, Pin.IN, Pin.PULL_DOWN)
# B6 = Pin(??, Pin.IN, Pin.PULL_DOWN)

button = Pin(15, Pin.IN, Pin.PULL_DOWN)

# Capture initial states for active encoders
prev_states[0] = (A1.value() << 1) | B1.value()
prev_states[1] = (A2.value() << 1) | B2.value()
# prev_states[2] = (A3.value() << 1) | B3.value()
# prev_states[3] = (A4.value() << 1) | B4.value()
# prev_states[4] = (A5.value() << 1) | B5.value()
# prev_states[5] = (A6.value() << 1) | B6.value()

# ── Encoder update (shared, called from IRQ) ───────────────────────────────────
def update_encoder(idx):
    global values, prev_states, lcdFlag
    if idx == 0:
        current = (A1.value() << 1) | B1.value()
    elif idx == 1:
        current = (A2.value() << 1) | B2.value()
    # elif idx == 2:
    #     current = (A3.value() << 1) | B3.value()
    # elif idx == 3:
    #     current = (A4.value() << 1) | B4.value()
    # elif idx == 4:
    #     current = (A5.value() << 1) | B5.value()
    # elif idx == 5:
    #     current = (A6.value() << 1) | B6.value()
    transition = (prev_states[idx] << 2) | current
    values[idx] = (values[idx] + TRANSITIONS[transition]) % 1000
    prev_states[idx] = current
    lcdFlag[idx] = True

def make_handler(idx):
    def handler(pin):
        update_encoder(idx)
    return handler

# ── Button / mode ──────────────────────────────────────────────────────────────
def switch(pin):
    global flag, values, mem
    if flag:
        values[0] = mem
        flag = 0
    else:
        mem = values[0]
        flag = 1

def modeNext(pin):
    global mode, modeFlag
    mode = (mode + 1) % len(MODES)
    modeFlag = True

def modePrev(pin):
    global mode, modeFlag
    mode = (mode - 1) % len(MODES)
    modeFlag = True

# ── Attach IRQs ───────────────────────────────────────────────────────────────
TRIGGER = Pin.IRQ_RISING | Pin.IRQ_FALLING

h1 = make_handler(0)
A1.irq(trigger=TRIGGER, handler=h1)
B1.irq(trigger=TRIGGER, handler=h1)

h2 = make_handler(1)
A2.irq(trigger=TRIGGER, handler=h2)
B2.irq(trigger=TRIGGER, handler=h2)

# h3 = make_handler(2)
# A3.irq(trigger=TRIGGER, handler=h3)
# B3.irq(trigger=TRIGGER, handler=h3)

# h4 = make_handler(3)
# A4.irq(trigger=TRIGGER, handler=h4)
# B4.irq(trigger=TRIGGER, handler=h4)

# h5 = make_handler(4)
# A5.irq(trigger=TRIGGER, handler=h5)
# B5.irq(trigger=TRIGGER, handler=h5)

# h6 = make_handler(5)
# A6.irq(trigger=TRIGGER, handler=h6)
# B6.irq(trigger=TRIGGER, handler=h6)

button.irq(trigger=Pin.IRQ_RISING, handler=modeNext)

# ── Main loop ──────────────────────────────────────────────────────────────────
while True:
    if any(lcdFlag):
        for i in range(6):
            if lcdFlag[i]:
                LCD.Number(values[i], 3, 0, 18 + 56 * i, 304, White, Black)
                lcdFlag[i] = False
    if modeFlag:
        LCD.Text(NAMES[mode], 354, 304, White, Black)
        modeFlag = False
    sleep_us(1)