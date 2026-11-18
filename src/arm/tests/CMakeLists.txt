cmake_minimum_required(VERSION 3.13)
project(servo_uart_tests LANGUAGES CXX)
enable_testing()

# Host-side tests of the actual transport, drivers and HAL against simulated
# GPIO/UART hardware. Deliberately independent of the cross-compiled firmware.
add_executable(servo_uart_tests
    half_duplex_test.cpp
    ../src/platform/pico_uart_transport.cpp
    ../src/drivers/servo_bus_driver.cpp
    ../src/drivers/xarm_controller_driver.cpp
    ../src/hal/servo_bus.cpp
)
target_compile_features(servo_uart_tests PRIVATE cxx_std_17)
target_compile_options(servo_uart_tests PRIVATE -Wall -Wextra -Werror -UNDEBUG)
target_include_directories(servo_uart_tests PRIVATE fakes ../src)
add_test(NAME servo_uart_tests COMMAND servo_uart_tests)
