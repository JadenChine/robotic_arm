from machine import Pin
from time import sleep_us

flag = 0
mem = 0

# ==============================
# SEGMENT PINS (COMMON ANODE)
# ==============================
seg_pins = [
    Pin(28, Pin.OUT),   # A
    Pin(27, Pin.OUT),   # B
    Pin(26, Pin.OUT),   # C
    Pin(22, Pin.OUT),   # D
    Pin(21, Pin.OUT),   # E
    Pin(20, Pin.OUT),   # F
    Pin(19, Pin.OUT),  # G
]

# ==============================
# DIGIT SELECT PINS
# ==============================
digits = [
    Pin(18, Pin.OUT),  # Digit 1 (hundreds)
    Pin(17, Pin.OUT),  # Digit 2 (tens)
    Pin(16, Pin.OUT),  # Digit 3 (ones)
]

# Turn all digits OFF
for d in digits:
    d.value(0)

# ==============================
# DIGIT PATTERNS (Common Anode)
# 0 = segment ON
# ==============================
DIGITS = [
    [0,0,0,0,0,0,1], # 0
    [1,0,0,1,1,1,1], # 1
    [0,0,1,0,0,1,0], # 2
    [0,0,0,0,1,1,0], # 3
    [1,0,0,1,1,0,0], # 4
    [0,1,0,0,1,0,0], # 5
    [0,1,0,0,0,0,0], # 6
    [0,0,0,1,1,1,1], # 7
    [0,0,0,0,0,0,0], # 8
    [0,0,0,0,1,0,0], # 9
]

def set_segments(value):
    pattern = DIGITS[value]
    for i in range(7):
        seg_pins[i].value(pattern[i])

# ==============================
# QUADRATURE ENCODER SETUP
# ==============================
A = Pin(2, Pin.IN, Pin.PULL_DOWN)
B = Pin(3, Pin.IN, Pin.PULL_DOWN)

button = Pin(4, Pin.IN, Pin.PULL_DOWN)

x = 0
prev_state = (A.value() << 1) | B.value()

# 4x resolution transition table
TRANSITIONS = [
    0,  -1,  1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
]

def update(pin):
    global x, prev_state

    current_state = (A.value() << 1) | B.value()
    index = (prev_state << 2) | current_state

    x += TRANSITIONS[index]

    # Keep number between 0–999
    if x < 0:
        x = 999
    elif x > 999:
        x = 0

    prev_state = current_state

def switch(pin):
    global flag, x, mem
    if flag:
        x = mem
        flag = 0
    else:
        mem = x
        flag = 1

A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update)
B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update)
button.irq(trigger=Pin.IRQ_RISING, handler=switch)

# ==============================
# MAIN LOOP (DISPLAY REFRESH)
# ==============================
while True:
    value = x

    hundreds = (value // 100) % 10
    tens = (value // 10) % 10
    ones = value % 10

    numbers = [hundreds, tens, ones]

    for i in range(3):

        # Turn ALL digits OFF first
        for d in digits:
            d.value(0)

        # Set segments for current digit
        set_segments(numbers[i])

        # Now enable the digit
        digits[i].value(1)

        sleep_us(2000)