from unittest import case
from machine import Pin
from time import sleep_us

flag = 0
mem = 0

FORWARD, INVERSE, TEACH = range(3)
MODES=[FORWARD, INVERSE, TEACH]

mode = FORWARD

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
    Pin(10, Pin.OUT), #1
    Pin(9, Pin.OUT),  #2
    Pin(8, Pin.OUT),  #3
    Pin(7, Pin.OUT),  #4
    Pin(6, Pin.OUT),  #5
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

def set_digits(digit):
    match (digit):
        case 0: # Group 1 Hundreds digit
            digits[0].value(1)
        case 1: # Group 1 Tens digit
            digits[1].value(1)
        case 2: # Group 1 Ones digit
            digits[0].value(1)
            digits[1].value(1)
        case 3: # Group 2 Hundreds digit
            digits[2].value(1)
        case 4: # Group 2 Tens digit
            digits[0].value(1)
            digits[2].value(1)
        case 5: # Group 2 Ones digit
            digits[1].value(1)
            digits[2].value(1)
        case 6: # Group 3 Hundreds digit
            digits[0].value(1)
            digits[1].value(1)
            digits[2].value(1)
        case 7: # Group 3 Tens digit
            digits[3].value(1)
        case 8: # Group 3 Ones digit
            digits[0].value(1)
            digits[3].value(1)
        case 9: # Group 4 Hundreds digit
            digits[1].value(1)
            digits[3].value(1)
        case 10: # Group 4 Tens digit
            digits[0].value(1)
            digits[1].value(1)
            digits[3].value(1)
        case 11: # Group 4 Ones digit
            digits[2].value(1)
            digits[3].value(1)
        case 12: # Group 5 Hundreds digit
            digits[0].value(1)
            digits[2].value(1)
            digits[3].value(1)
        case 13: # Group 5 Tens digit
            digits[1].value(1)
            digits[2].value(1)
            digits[3].value(1)
        case 14: # Group 5 Ones digit
            digits[0].value(1)
            digits[1].value(1)
            digits[2].value(1)
            digits[3].value(1)
        case 15: # Group 6 Hundreds digit
            digits[4].value(1)
        case 16: # Group 6 Tens digit
            digits[0].value(1)
            digits[4].value(1)
        case 17: # Group 6 Ones digit
            digits[1].value(1)
            digits[4].value(1)
        case _:
            pass

    

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
nextMode = Pin(14, Pin.IN, Pin.PULL_DOWN)
prevMode = Pin(15, Pin.IN, Pin.PULL_DOWN)

x = 0
prev_state = (A.value() << 1) | B.value()

# 4x resolution transition table
TRANSITIONS = [
    0,  -1,  1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
]

def encoder1(pin):
    global value1, prev_state1
    value1, prev_state1 = update(value1, prev_state1)

def encoder2(pin):
    global value2, prev_state2
    value2, prev_state2 = update(value2, prev_state2)

def encoder3(pin):
    global value3, prev_state3
    value3, prev_state3 = update(value3, prev_state3)

def encoder4(pin):
    global value4, prev_state4
    value4, prev_state4 = update(value4, prev_state4)

def encoder5(pin):
    global value5, prev_state5
    value5, prev_state5 = update(value5, prev_state5)

def encoder6(pin):
    global value6, prev_state6
    value6, prev_state6 = update(value6, prev_state6)

def update(value, prev):

    current_state = (A.value() << 1) | B.value()
    index = (prev_state << 2) | current_state

    value += TRANSITIONS[index]

    # Keep number between 0–999
    if value < 0:
        value = 999
    elif value > 999:
        value = 0

    prev_state = current_state

    return value, prev_state

def switch(pin):
    global flag, x, mem
    if flag:
        x = mem
        flag = 0
    else:
        mem = x
        flag = 1
        
def modeNext(pin):
    global mode
    mode = (mode + 1) % len(MODES)
    
def modePrev(pin):
    global mode
    mode = (mode - 1) % len(MODES)

A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder1)
B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder1)
button.irq(trigger=Pin.IRQ_RISING, handler=switch)

nextMode.irq(trigger=Pin.IRQ_RISING, handler=modeNext)
prevMode.irq(trigger=Pin.IRQ_RISING, handler=modePrev)

def split_number(num):
    return [num // 100, (num % 100) // 10, num % 10]

# ==============================
# MAIN LOOP (DISPLAY REFRESH)
# ==============================
while True:
    print(mode)


    # Display on each of the 6 7 segment displays
    for i in range(18):

        # Turn ALL digits OFF first
        for d in digits:
            d.value(0)

        # Set segments for current digit
        set_segments(numbers[i])

        # Now enable the digit
        set_digits(i)

        sleep_us(2000)