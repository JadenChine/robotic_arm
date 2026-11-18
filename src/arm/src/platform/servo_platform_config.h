#ifndef ARM_SERVO_PLATFORM_CONFIG_H
#define ARM_SERVO_PLATFORM_CONFIG_H

#define SERVO_BUS_DEFAULT_BACKEND 0u // SERVO_BUS_BACKEND_DIRECT_SERVO

#define SERVO_BUS_DEFAULT_UART_INDEX 0
#define SERVO_BUS_DIRECT_DEFAULT_BAUD_RATE 115200u
#define SERVO_BUS_CONTROLLER_DEFAULT_BAUD_RATE 9600u
// UART pins use Pico GPIO numbers.
#define SERVO_BUS_DEFAULT_TX_PIN 0u // UART0 TX, header pin 1
#define SERVO_BUS_DEFAULT_RX_PIN 1u // UART0 RX, header pin 2
// Active-HIGH buffer enables for the direct bus.
#define SERVO_BUS_DIRECT_DEFAULT_TX_ENABLE_PIN 2u // TX_EN, header pin 4
#define SERVO_BUS_DIRECT_DEFAULT_RX_ENABLE_PIN 3u // RX_EN, header pin 5

#define SERVO_BUS_DEFAULT_SERVO_ID 1u
#define SERVO_BUS_DEFAULT_MOVE_TIME_MS 700u
#define SERVO_BUS_DEFAULT_STEP_DEGREES 15.0f

// Status LED pins use Pico GPIO numbers.
#define BOARD_HEARTBEAT_LED_PIN 17u // Yellow, header pin 22
#define BOARD_POWER_LED_PIN 18u // Green, header pin 24
#define BOARD_BLUETOOTH_LED_PIN 19u // Blue, header pin 25
#define BOARD_JOINT_MODE_LED_PIN 20u // Blue, header pin 26
#define BOARD_CARTESIAN_MODE_LED_PIN 21u // Blue, header pin 27
#define BOARD_STATUS_LED_ACTIVE_LEVEL 1u
#define BOARD_STATUS_LED_MASK ((1u << BOARD_HEARTBEAT_LED_PIN) | \
                               (1u << BOARD_POWER_LED_PIN) | \
                               (1u << BOARD_BLUETOOTH_LED_PIN) | \
                               (1u << BOARD_JOINT_MODE_LED_PIN) | \
                               (1u << BOARD_CARTESIAN_MODE_LED_PIN))
#define BOARD_HEARTBEAT_INTERVAL_MS 500u

#endif
