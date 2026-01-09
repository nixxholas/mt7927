#!/bin/bash
# MT7927 WiFi Driver Installation Script
# This script attempts to enable MT7927 support using the existing mt7925 driver

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}MT7927 WiFi Driver Installation${NC}"
echo "================================"
echo ""

# Check kernel version
KERNEL=$(uname -r)
echo "Running kernel: $KERNEL"

# Check if modules directory exists
if [ ! -d "/lib/modules/$KERNEL/kernel" ]; then
    echo -e "${RED}ERROR: Kernel modules for $KERNEL not found!${NC}"
    echo "You need to reboot to a kernel that has matching modules."
    echo ""
    echo "Available kernels with modules:"
    ls /lib/modules/
    exit 1
fi

# Check for MT7927 device
echo ""
echo "Checking for MT7927 device..."
if ! lspci -nn | grep -q "14c3:7927"; then
    echo -e "${YELLOW}WARNING: MT7927 device not detected${NC}"
    echo "This script is for MediaTek MT7927 (PCI ID: 14c3:7927)"
    exit 1
fi
echo -e "${GREEN}MT7927 device found${NC}"

# Check if mt7925e module exists
echo ""
echo "Checking for mt7925e module..."
if ! modinfo mt7925e &>/dev/null; then
    echo -e "${RED}ERROR: mt7925e module not found${NC}"
    echo "The kernel doesn't have the mt7925 driver."
    exit 1
fi
echo -e "${GREEN}mt7925e module found${NC}"

# Check firmware
echo ""
echo "Checking firmware files..."
FW_DIR="/lib/firmware/mediatek/mt7925"
if [ ! -f "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin" ] && [ ! -f "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin.zst" ]; then
    echo -e "${YELLOW}WARNING: MT7925 firmware not found${NC}"
    echo "Installing firmware..."
    sudo mkdir -p "$FW_DIR"
    # Try to decompress if zst exists
    if [ -f "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin.zst" ]; then
        sudo zstd -d "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin.zst" -o "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin"
    fi
fi
echo -e "${GREEN}Firmware available${NC}"

# Load dependencies
echo ""
echo "Loading mt76 driver dependencies..."
sudo modprobe mt76 || true
sudo modprobe mt76-connac-lib || true
sudo modprobe mt792x-lib || true
sudo modprobe mt7925-common || true

# Load mt7925e
echo "Loading mt7925e driver..."
sudo modprobe mt7925e

# Check if already bound
if lspci -k 2>/dev/null | grep -A2 "14c3:7927" | grep -q "Kernel driver in use"; then
    DRIVER=$(lspci -k 2>/dev/null | grep -A2 "14c3:7927" | grep "Kernel driver" | awk '{print $NF}')
    echo -e "${GREEN}Device already bound to driver: $DRIVER${NC}"

    # Check for wlan interface
    if ip link show | grep -q wlan; then
        echo -e "${GREEN}WiFi interface available!${NC}"
        ip link show | grep wlan
        exit 0
    fi
fi

# Try new_id approach
echo ""
echo "Attempting to bind MT7927 to mt7925e driver..."
echo "Method: sysfs new_id"

if [ -f /sys/bus/pci/drivers/mt7925e/new_id ]; then
    echo "14c3 7927" | sudo tee /sys/bus/pci/drivers/mt7925e/new_id 2>/dev/null && {
        echo -e "${GREEN}new_id accepted!${NC}"
        sleep 2
    } || {
        echo -e "${YELLOW}new_id failed (this is expected if device is already bound)${NC}"
    }
fi

# Unbind from any existing driver and rebind
PCI_DEV=$(lspci -D | grep "14c3:7927" | awk '{print $1}')
if [ -n "$PCI_DEV" ]; then
    echo "PCI device: $PCI_DEV"

    # Check current driver
    CURRENT_DRIVER=""
    if [ -L "/sys/bus/pci/devices/$PCI_DEV/driver" ]; then
        CURRENT_DRIVER=$(basename $(readlink "/sys/bus/pci/devices/$PCI_DEV/driver"))
        echo "Current driver: $CURRENT_DRIVER"
    fi

    # If not bound to mt7925e, try to bind
    if [ "$CURRENT_DRIVER" != "mt7925e" ]; then
        echo "Attempting driver bind..."

        # Unbind if bound to something else
        if [ -n "$CURRENT_DRIVER" ]; then
            echo "$PCI_DEV" | sudo tee "/sys/bus/pci/drivers/$CURRENT_DRIVER/unbind" 2>/dev/null || true
            sleep 1
        fi

        # Try bind
        echo "$PCI_DEV" | sudo tee /sys/bus/pci/drivers/mt7925e/bind 2>/dev/null && {
            echo -e "${GREEN}Successfully bound to mt7925e!${NC}"
        } || {
            echo -e "${YELLOW}Direct bind failed${NC}"
        }
    fi
fi

# Check final state
echo ""
echo "Final status:"
sleep 2

if lspci -k 2>/dev/null | grep -A2 "14c3:7927" | grep -q "mt7925e"; then
    echo -e "${GREEN}SUCCESS: MT7927 bound to mt7925e driver!${NC}"

    # Check for wlan interface
    sleep 2
    if ip link show | grep -q wlan; then
        echo -e "${GREEN}WiFi interface created!${NC}"
        ip link show | grep wlan
        echo ""
        echo "To scan for networks:"
        echo "  iwctl station wlan0 scan"
        echo "  iwctl station wlan0 get-networks"
    else
        echo -e "${YELLOW}WiFi interface not created yet. Check dmesg for errors:${NC}"
        echo "  sudo dmesg | tail -30"
    fi
else
    echo -e "${RED}FAILED: Could not bind MT7927 to mt7925e${NC}"
    echo ""
    echo "The kernel driver may need patching. Options:"
    echo "1. Build mt76 from source with MT7927 support"
    echo "2. Wait for upstream kernel support"
    echo ""
    echo "Check kernel messages for details:"
    echo "  sudo dmesg | grep -i mt79"
    exit 1
fi
