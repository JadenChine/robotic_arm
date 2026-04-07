# ============================================================================
# CMake Configuration for Raspberry Pi Pico Robotic Arm Project
# ============================================================================

# Specify minimum CMake version (Pico SDK requires 3.13+)
cmake_minimum_required(VERSION 3.13)

# Use the pico-sdk submodule located next to this project directory.
# This keeps the project self-contained and avoids relying on a global SDK path.
set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../pico-sdk")
include(${CMAKE_CURRENT_LIST_DIR}/../pico-sdk/external/pico_sdk_import.cmake)

# Define project name and enabled languages.
project(robotic_arm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK (sets up toolchain, libraries, and build helpers).
pico_sdk_init()


# ============================================================================
# Source File Collection
# ============================================================================

# Collect project source files:
# - main.c at the project root
# - all C/C++ files inside src/
# CONFIGURE_DEPENDS tells CMake to re-check this glob when files are added/removed.
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/main.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
)

# Create the firmware executable from the collected source files.
add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})


# ============================================================================
# Library Dependencies
# ============================================================================

# Link required Pico libraries.
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_uart
)

# Add include directories used by this target.
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)


# ============================================================================
# Standard I/O Configuration
# ============================================================================

# Enable USB serial for printf/debug output.
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Disable UART serial output.
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate additional output files (.uf2, .bin, .hex, etc.).
pico_add_extra_outputs(${PROJECT_NAME})