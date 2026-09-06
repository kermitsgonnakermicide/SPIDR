#!/usr/bin/env bash
# calibrate_oakd.sh - Placeholder for OAK-D Lite stereo calibration
# Usage: ./calibrate_oakd.sh
#
# This is a placeholder script. For actual calibration, use the official
# Luxonis calibration tool:
#   https://github.com/luxonis/depthai-calibration
#
# Steps:
# 1. Print a calibration checkerboard pattern
# 2. Run depthai-calibration to capture images at various angles
# 3. The tool generates a calibration JSON file
# 4. The driver will automatically load the calibration from the OAK-D device

set -euo pipefail

CALIB_DIR="${HOME}/.depthai_calibration"
mkdir -p "$CALIB_DIR"

echo "OAK-D Lite Calibration Helper"
echo "============================="
echo ""
echo "1. Print a calibration checkerboard (7x9 or 8x6 squares recommended)"
echo "   Download from: https://docs.opencv.org/4.x/pattern.png"
echo ""
echo "2. Clone the calibration tool:"
echo "   git clone https://github.com/luxonis/depthai-calibration.git"
echo "   cd depthai-calibration"
echo ""
echo "3. Run the calibration tool with your OAK-D Lite:"
echo "   python3 calibrate.py --board <board_type> --rows <rows> --cols <cols>"
echo ""
echo "4. The calibration JSON will be saved to your OAK-D device's flash."
echo ""
echo "Calibration files are stored in: $CALIB_DIR"
echo ""
echo "Note: Factory calibration is typically sufficient for most experiments."
echo "Re-calibration is only needed for high-precision applications."
