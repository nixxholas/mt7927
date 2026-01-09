#!/bin/bash
# Complete MT7927 WiFi Setup Script
# This script configures MT7927 WiFi to work with iwd

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}  MT7927 WiFi Complete Setup${NC}"
echo -e "${BLUE}====================================${NC}"
echo ""

# Step 1: Check kernel
echo -e "${BLUE}Step 1: Checking kernel...${NC}"
KERNEL=$(uname -r)
echo "Running kernel: $KERNEL"

if [ ! -d "/lib/modules/$KERNEL/kernel" ]; then
    echo -e "${RED}ERROR: Kernel modules for $KERNEL not found!${NC}"
    echo ""
    echo "Available kernels:"
    ls /lib/modules/
    echo ""
    echo -e "${YELLOW}Please reboot to use a kernel with available modules.${NC}"
    echo "After reboot, run this script again."
    exit 1
fi
echo -e "${GREEN}Kernel modules available${NC}"
echo ""

# Step 2: Check device
echo -e "${BLUE}Step 2: Checking for MT7927 device...${NC}"
if ! lspci -nn | grep -q "14c3:7927"; then
    echo -e "${RED}ERROR: MT7927 device not detected${NC}"
    exit 1
fi
echo -e "${GREEN}MT7927 device found:${NC}"
lspci -nn | grep "14c3:7927"
echo ""

# Step 3: Check/install firmware
echo -e "${BLUE}Step 3: Checking firmware...${NC}"
FW_DIR="/lib/firmware/mediatek/mt7925"
if [ ! -d "$FW_DIR" ]; then
    echo "Firmware directory not found, creating..."
    sudo mkdir -p "$FW_DIR"
fi

# Decompress firmware if needed
for f in "$FW_DIR"/*.bin.zst; do
    if [ -f "$f" ]; then
        BASENAME=$(basename "$f" .zst)
        if [ ! -f "$FW_DIR/$BASENAME" ]; then
            echo "Decompressing $BASENAME..."
            sudo zstd -d "$f" -o "$FW_DIR/$BASENAME" 2>/dev/null || true
        fi
    fi
done

if [ -f "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin" ] || [ -f "$FW_DIR/WIFI_RAM_CODE_MT7925_1_1.bin.zst" ]; then
    echo -e "${GREEN}Firmware available${NC}"
else
    echo -e "${YELLOW}WARNING: Firmware may be missing${NC}"
    echo "Expected files in $FW_DIR:"
    echo "  - WIFI_RAM_CODE_MT7925_1_1.bin"
    echo "  - WIFI_MT7925_PATCH_MCU_1_1_hdr.bin"
fi
echo ""

# Step 4: Load mt7925e driver
echo -e "${BLUE}Step 4: Loading mt7925e driver...${NC}"
if ! modinfo mt7925e &>/dev/null; then
    echo -e "${RED}ERROR: mt7925e module not available${NC}"
    exit 1
fi

# Load driver stack
sudo modprobe mt76 2>/dev/null || true
sudo modprobe mt76-connac-lib 2>/dev/null || true
sudo modprobe mt792x-lib 2>/dev/null || true
sudo modprobe mt7925-common 2>/dev/null || true
sudo modprobe mt7925e

if lsmod | grep -q mt7925e; then
    echo -e "${GREEN}mt7925e driver loaded${NC}"
else
    echo -e "${RED}Failed to load mt7925e${NC}"
    exit 1
fi
echo ""

# Step 5: Try to bind MT7927
echo -e "${BLUE}Step 5: Binding MT7927 to driver...${NC}"

# Check current binding
PCI_DEV=$(lspci -D | grep "14c3:7927" | awk '{print $1}')
echo "PCI device: $PCI_DEV"

CURRENT_DRIVER=""
if [ -L "/sys/bus/pci/devices/$PCI_DEV/driver" ]; then
    CURRENT_DRIVER=$(basename $(readlink "/sys/bus/pci/devices/$PCI_DEV/driver"))
fi

if [ "$CURRENT_DRIVER" = "mt7925e" ]; then
    echo -e "${GREEN}Already bound to mt7925e${NC}"
else
    # Try new_id first
    echo "Trying new_id approach..."
    echo "14c3 7927" | sudo tee /sys/bus/pci/drivers/mt7925e/new_id 2>/dev/null && {
        echo -e "${GREEN}new_id succeeded${NC}"
        sleep 1
    } || {
        echo "new_id failed, trying DKMS shim..."

        # Check if DKMS module exists
        if modinfo mt7927_shim &>/dev/null; then
            sudo modprobe mt7927_shim
        else
            # Build and install DKMS module
            echo "Building DKMS module..."
            if [ -d "$SCRIPT_DIR/dkms" ]; then
                bash "$SCRIPT_DIR/dkms/install-dkms.sh"
                sudo modprobe mt7927_shim
            else
                echo -e "${RED}DKMS directory not found${NC}"
            fi
        fi
    }

    # Manual bind attempt
    if [ -n "$CURRENT_DRIVER" ] && [ "$CURRENT_DRIVER" != "mt7925e" ]; then
        echo "Unbinding from $CURRENT_DRIVER..."
        echo "$PCI_DEV" | sudo tee "/sys/bus/pci/drivers/$CURRENT_DRIVER/unbind" 2>/dev/null || true
        sleep 1
    fi

    echo "Binding to mt7925e..."
    echo "$PCI_DEV" | sudo tee /sys/bus/pci/drivers/mt7925e/bind 2>/dev/null || true
    sleep 2
fi

# Check final binding
if [ -L "/sys/bus/pci/devices/$PCI_DEV/driver" ]; then
    FINAL_DRIVER=$(basename $(readlink "/sys/bus/pci/devices/$PCI_DEV/driver"))
    if [ "$FINAL_DRIVER" = "mt7925e" ]; then
        echo -e "${GREEN}Successfully bound to mt7925e!${NC}"
    else
        echo -e "${YELLOW}Bound to: $FINAL_DRIVER${NC}"
    fi
else
    echo -e "${RED}Device not bound to any driver${NC}"
fi
echo ""

# Step 6: Check for WiFi interface
echo -e "${BLUE}Step 6: Checking WiFi interface...${NC}"
sleep 2

WLAN_IF=""
for iface in /sys/class/net/wlan*; do
    if [ -e "$iface" ]; then
        WLAN_IF=$(basename "$iface")
        break
    fi
done

if [ -n "$WLAN_IF" ]; then
    echo -e "${GREEN}WiFi interface found: $WLAN_IF${NC}"

    # Bring interface up
    sudo ip link set "$WLAN_IF" up 2>/dev/null || true
    echo ""

    # Step 7: Configure iwd
    echo -e "${BLUE}Step 7: Configuring iwd...${NC}"

    # Check if iwd is installed
    if ! command -v iwctl &>/dev/null; then
        echo "Installing iwd..."
        sudo pacman -S --noconfirm iwd || {
            echo -e "${YELLOW}Could not install iwd automatically${NC}"
            echo "Install manually: sudo pacman -S iwd"
        }
    fi

    # Enable and start iwd
    sudo systemctl enable iwd 2>/dev/null || true
    sudo systemctl start iwd 2>/dev/null || true

    if systemctl is-active iwd &>/dev/null; then
        echo -e "${GREEN}iwd is running${NC}"
    else
        echo -e "${YELLOW}iwd may not be running${NC}"
    fi
    echo ""

    # Final instructions
    echo -e "${GREEN}====================================${NC}"
    echo -e "${GREEN}  Setup Complete!${NC}"
    echo -e "${GREEN}====================================${NC}"
    echo ""
    echo "To connect to WiFi using iwd:"
    echo ""
    echo "  iwctl"
    echo "  [iwd]# station $WLAN_IF scan"
    echo "  [iwd]# station $WLAN_IF get-networks"
    echo "  [iwd]# station $WLAN_IF connect YOUR_NETWORK_NAME"
    echo ""
    echo "Or use iwctl directly:"
    echo "  iwctl station $WLAN_IF connect YOUR_NETWORK_NAME"
    echo ""

    # Make persistent
    echo -e "${BLUE}Making configuration persistent...${NC}"

    # Create module load config
    echo "mt7925e" | sudo tee /etc/modules-load.d/mt7927.conf >/dev/null
    if modinfo mt7927_shim &>/dev/null 2>&1; then
        echo "mt7927_shim" | sudo tee -a /etc/modules-load.d/mt7927.conf >/dev/null
    fi

    # Install udev rule
    if [ -f "$SCRIPT_DIR/dkms/99-mt7927.rules" ]; then
        sudo cp "$SCRIPT_DIR/dkms/99-mt7927.rules" /etc/udev/rules.d/
        sudo udevadm control --reload-rules
        echo "udev rules installed"
    fi

    echo -e "${GREEN}Configuration saved. WiFi should work after reboot.${NC}"

else
    echo -e "${RED}No WiFi interface created${NC}"
    echo ""
    echo "Check kernel messages for errors:"
    echo "  sudo dmesg | grep -i 'mt79\|firmware\|wlan'"
    echo ""
    echo "The driver may need additional patches for MT7927."
    exit 1
fi
