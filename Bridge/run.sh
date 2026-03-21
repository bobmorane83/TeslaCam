#!/bin/bash
# ESP32 LED Blink Project - Quick Commands

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORMIO="platformio"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

function print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

function print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

function print_error() {
    echo -e "${RED}✗ $1${NC}"
}

function print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

# Command functions
build() {
    print_header "Building Project"
    $PLATFORMIO run -e esp32dev
    print_success "Build complete"
}

upload() {
    print_header "Uploading to ESP32"
    $PLATFORMIO run -e esp32dev -t upload
    print_success "Upload complete"
}

monitor() {
    print_header "Opening Serial Monitor"
    print_info "Press Ctrl+C to exit"
    $PLATFORMIO device monitor -b 115200
}

upload_and_monitor() {
    print_header "Building, Uploading, and Monitoring"
    build
    upload
    monitor
}

clean() {
    print_header "Cleaning Build Artifacts"
    $PLATFORMIO run -e esp32dev -t clean
    rm -rf .pio/
    print_success "Clean complete"
}

simulate() {
    print_header "Running LED Blink Simulator"
    python3 "$PROJECT_DIR/simulate_blink.py"
}

list_ports() {
    print_header "Available Serial Ports"
    $PLATFORMIO device list
}

info() {
    print_header "Project Information"
    echo "Project: ESP32 LED Blink"
    echo "Board: NodeMCU-ESP32 DEVKITV1"
    echo "Framework: Arduino"
    echo "LED Pin: GPIO 2"
    echo "Blink Interval: 1 second"
    echo ""
    echo "Available Commands:"
    echo "  ./run.sh build              - Build the project"
    echo "  ./run.sh upload             - Upload to ESP32"
    echo "  ./run.sh monitor            - Open serial monitor (115200 baud)"
    echo "  ./run.sh upload_and_monitor - Upload and open monitor"
    echo "  ./run.sh clean              - Clean build artifacts"
    echo "  ./run.sh simulate           - Run simulator (no hardware needed)"
    echo "  ./run.sh list_ports         - List available COM ports"
    echo "  ./run.sh info               - Show this information"
}

# Main
if [ $# -eq 0 ]; then
    info
else
    case "$1" in
        build)
            build
            ;;
        upload)
            upload
            ;;
        monitor)
            monitor
            ;;
        upload_and_monitor)
            upload_and_monitor
            ;;
        clean)
            clean
            ;;
        simulate)
            simulate
            ;;
        list_ports)
            list_ports
            ;;
        info)
            info
            ;;
        *)
            print_error "Unknown command: $1"
            echo ""
            info
            exit 1
            ;;
    esac
fi
