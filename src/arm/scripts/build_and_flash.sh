#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd -- "${SCRIPT_DIR}/.." && pwd)
BUILD_DIR="${PROJECT_DIR}/build"
PROJECT_NAME=$(grep -E '^[[:space:]]*project\(' "${PROJECT_DIR}/CMakeLists.txt" | sed -n 's/^project(\([^ )]*\).*/\1/p')

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

CONNECT=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--connect)
            CONNECT=1
            shift
            ;;
        -h|--help)
            cat <<'EOF'
Usage: build_and_flash.sh [--connect]

Builds the Pico project from the local build directory and flashes the resulting UF2.
EOF
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown argument: $1${NC}"
            exit 1
            ;;
    esac
done

if [[ -z "${PROJECT_NAME}" ]]; then
    echo -e "${RED}Error: Could not read project name from CMakeLists.txt${NC}"
    exit 1
fi

if ! command -v picotool >/dev/null 2>&1; then
    echo -e "${RED}Error: picotool is not installed or not on PATH${NC}"
    exit 1
fi

if [[ ! -d "${BUILD_DIR}" ]]; then
    echo -e "${RED}Error: Build directory not found at ${BUILD_DIR}${NC}"
    echo "Run: cmake -S ${PROJECT_DIR} -B ${BUILD_DIR}"
    exit 1
fi

echo "Building project: ${PROJECT_NAME}"
cmake --build "${BUILD_DIR}"

UF2_FILE="${BUILD_DIR}/${PROJECT_NAME}.uf2"
if [[ ! -f "${UF2_FILE}" ]]; then
    echo -e "${RED}Error: Expected output file not found: ${UF2_FILE}${NC}"
    exit 1
fi

echo -e "${GREEN}Build successful!${NC}"
echo "Flashing file: ${UF2_FILE}"

if picotool load -x "${UF2_FILE}" -F; then
    echo -e "${GREEN}Flash successful!${NC}"
    if [[ ${CONNECT} -eq 1 ]]; then
        echo -e "\n${GREEN}Connecting to Pico serial...${NC}"
        "${SCRIPT_DIR}/serial_connect.sh"
    fi
else
    echo -e "${RED}Error: Could not flash to device. Is the board connected in BOOTSEL mode?${NC}"
    exit 1
fi
