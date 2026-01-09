#!/bin/bash
# Install MT7927 DKMS shim module

set -e

DKMS_NAME="mt7927"
DKMS_VERSION="1.0"
SRC_DIR="$(dirname "$0")/mt7927-dkms-1.0"

echo "Installing MT7927 DKMS module..."

# Check for DKMS
if ! command -v dkms &> /dev/null; then
    echo "ERROR: DKMS not installed. Install with: sudo pacman -S dkms"
    exit 1
fi

# Check for kernel headers
KERNEL=$(uname -r)
if [ ! -d "/lib/modules/$KERNEL/build" ]; then
    echo "ERROR: Kernel headers not found for $KERNEL"
    echo "Install with: sudo pacman -S linux-headers"
    exit 1
fi

# Remove existing installation if present
if dkms status "$DKMS_NAME/$DKMS_VERSION" &>/dev/null; then
    echo "Removing existing DKMS module..."
    sudo dkms remove "$DKMS_NAME/$DKMS_VERSION" --all || true
fi

# Copy source to DKMS tree
DKMS_SRC="/usr/src/$DKMS_NAME-$DKMS_VERSION"
echo "Copying source to $DKMS_SRC..."
sudo rm -rf "$DKMS_SRC"
sudo mkdir -p "$DKMS_SRC"
sudo cp "$SRC_DIR"/* "$DKMS_SRC/"

# Add to DKMS
echo "Adding module to DKMS..."
sudo dkms add "$DKMS_NAME/$DKMS_VERSION"

# Build
echo "Building module..."
sudo dkms build "$DKMS_NAME/$DKMS_VERSION"

# Install
echo "Installing module..."
sudo dkms install "$DKMS_NAME/$DKMS_VERSION"

echo ""
echo "DKMS module installed successfully!"
echo ""
echo "To load the module:"
echo "  sudo modprobe mt7925e"
echo "  sudo modprobe mt7927_shim"
echo ""
echo "To auto-load at boot, create /etc/modules-load.d/mt7927.conf:"
echo "  echo -e 'mt7925e\nmt7927_shim' | sudo tee /etc/modules-load.d/mt7927.conf"
