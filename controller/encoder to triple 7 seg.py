from machine import Pin
from time import sleep_us

flag = 0
mem = 0
value1 = value2 = value3 = value4 = value5 = value6 = 0
FORWARD, INVERSE, TEACH = range(3)
MODES=[FORWARD, INVERSE, TEACH]

mode = FORWARD

numbers = [
    [0, 0, 0],  # Group 1
    [0, 0, 0],  # Group 2
    [0, 0, 0],  # Group 3
    [0, 0, 0],  # Group 4
    [0, 0, 0],  # Group 5
    [0, 0, 0]   # Group 6
]

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
    Pin(5, Pin.OUT),  #6
    Pin(18, Pin.OUT),  #4
    Pin(12, Pin.OUT),  #5
    Pin(11, Pin.OUT),  #6
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
        if digit == 0: # Group 1 Hundreds digit
            digits[0].value(1)
        elif digit == 1: # Group 1 Tens digit
            digits[1].value(1)
        elif digit == 2: # Group 1 Ones digit
            digits[2].value(1)
            #digits[0].value(1)
            #digits[1].value(1)
        elif digit == 3: # Group 2 Hundreds digit
            digits[3].value(1)
            #digits[2].value(1)
        elif digit == 4: # Group 2 Tens digit
            digits[4].value(1)
            #digits[0].value(1)
            #digits[2].value(1)
        elif digit == 5: # Group 2 Ones digit
            digits[5].value(1)
            #digits[1].value(1)
            #digits[2].value(1)
        elif digit == 6: # Group 3 Hundreds digit
            digits[6].value(1)
            #digits[0].value(1)
            #digits[1].value(1)
            #digits[2].value(1)
        elif digit == 7: # Group 3 Tens digit
            digits[7].value(1)
            #digits[3].value(1)
        elif digit == 8: # Group 3 Ones digit
            digits[8].value(1)
            #digits[0].value(1)
            #digits[3].value(1)
        else:
            pass
        '''
        elif digit == 9: # Group 4 Hundreds digit
            digits[1].value(1)
            digits[3].value(1)
        elif digit == 10: # Group 4 Tens digit
            digits[0].value(1)
            digits[1].value(1)
            digits[3].value(1)
        elif digit == 11: # Group 4 Ones digit
            digits[2].value(1)
            digits[3].value(1)
        elif digit == 12: # Group 5 Hundreds digit
            digits[0].value(1)
            digits[2].value(1)
            digits[3].value(1)
        elif digit == 13: # Group 5 Tens digit
            digits[1].value(1)
            digits[2].value(1)
            digits[3].value(1)
        elif digit == 14: # Group 5 Ones digit
            digits[0].value(1)
            digits[1].value(1)
            digits[2].value(1)
            digits[3].value(1)
        elif digit == 15: # Group 6 Hundreds digit
            digits[4].value(1)
        elif digit == 16: # Group 6 Tens digit
            digits[0].value(1)
            digits[4].value(1)
        elif digit == 17: # Group 6 Ones digit
            digits[1].value(1)
            digits[4].value(1)
        '''

def set_segments(value):
    pattern = DIGITS[value]
    for i in range(7):
        seg_pins[i].value(pattern[i])

# ==============================
# QUADRATURE ENCODER SETUP
# ==============================
A1 = Pin(2, Pin.IN, Pin.PULL_DOWN)
B1 = Pin(3, Pin.IN, Pin.PULL_DOWN)

A2 = Pin(16, Pin.IN, Pin.PULL_DOWN)
B2 = Pin(17, Pin.IN, Pin.PULL_DOWN)

button = Pin(4, Pin.IN, Pin.PULL_DOWN)

prev_state1 = (A1.value() << 1) | B1.value()
prev_state2 = (A2.value() << 1) | B2.value()

# 4x resolution transition table
TRANSITIONS = [
    0,  -1,  1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
]

def encoder1(pin):
    global value1, prev_state1
    value1, prev_state1 = update(value1, prev_state1, 0)

def encoder2(pin):
    global value2, prev_state2
    value2, prev_state2 = update(value2, prev_state2, 1)

def encoder3(pin):
    global value3, prev_state3
    value3, prev_state3 = update(value3, prev_state3, 2)

def encoder4(pin):
    global value4, prev_state4
    value4, prev_state4 = update(value4, prev_state4, 3)

def encoder5(pin):
    global value5, prev_state5
    value5, prev_state5 = update(value5, prev_state5, 4)

def encoder6(pin):
    global value6, prev_state6
    value6, prev_state6 = update(value6, prev_state6, 5)

def update(value, prev_state, num):

    current_state = (Encoder_values[num][0] << 1) | Encoder_values[num][1]
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
    global flag, value1, mem
    if flag:
        value1 = mem
        flag = 0
    else:
        mem = value1
        flag = 1
    print(pin)
        
def modeNext(pin):
    global mode
    mode = (mode + 1) % len(MODES)
    
def modePrev(pin):
    global mode
    mode = (mode - 1) % len(MODES)

A1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder1)
B1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder1)
A2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder2)
B2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder2)
button.irq(trigger=Pin.IRQ_RISING, handler=switch)

def split_number(num):
    return [num // 100, (num % 100) // 10, num % 10]

# ==============================
# MAIN LOOP (DISPLAY REFRESH)
# ==============================
while True:
    Encoder_values = [
    [A1.value(), B1.value()],
    [A2.value(), B2.value()]
    ]
    
    # Update numbers list with current encoder values
    numbers[0] = split_number(value1)
    numbers[1] = split_number(value2)
    numbers[2] = split_number(value3)
    numbers[3] = split_number(value4)
    numbers[4] = split_number(value5)
    numbers[5] = split_number(value6)

    # Display on each of the 6 7 segment displays
    for displays in range(6):
        for digit in range(3):
            # Turn ALL digits OFF first
            for d in digits:
                d.value(0)
            # Set segments for current digit
            set_segments(numbers[displays][digit])

            # Now enable the digit
            set_digits(displays * 3 + digit)

            sleep_us(1)