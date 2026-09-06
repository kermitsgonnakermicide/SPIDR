#!/usr/bin/env bash
# setup_oakd_permissions.sh - Setup USB permissions for OAK-D Lite camera
# Usage: sudo ./setup_oakd_permissions.sh
#
# This script creates a udev rule for the OAK-D camera's USB device,
# allowing non-root users to access the camera without needing sudo.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

UDEV_RULES_FILE="/etc/udev/rules.d/80-oakd-lite.rules"

echo "Setting up OAK-D Lite USB permissions..."

# Create udev rule for OAK-D Lite (and other Luxonis devices)
cat > "$UDEV_RULES_FILE" <<EOF
# OAK-D Lite and other Luxonis device permissions
# Allow non-root users to access OAK-D cameras via USB
SUBSYSTEM=="usb", ATTR{idVendor}=="03e7", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="1d6b", MODE="0666", GROUP="plugdev"
EOF

echo "udev rule created: $UDEV_RULES_FILE"
echo "Reloading udev rules..."

udevadm control --reload-rules
udevadm trigger

# Add current user to plugdev group if not already
if [[ -n "${SUDO_USER:-}" ]]; then
    USER_NAME="$SUDO_USER"
    if ! groups "$USER_NAME" | grep -q "plugdev"; then
        echo "Adding user '$USER_NAME' to plugdev group..."
        usermod -a -G plugdev "$USER_NAME"
        echo "Note: You may need to log out and log back in for the group change to take effect."
    fi
fi

echo ""
echo "OAK-D Lite USB permissions setup complete."
echo "You can now run the OAK-D driver without sudo:"
echo "  ros2 launch spooder_perception oakd_driver.launch.py"
