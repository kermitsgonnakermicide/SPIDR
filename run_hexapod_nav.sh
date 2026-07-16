#!/usr/bin/env bash
# =============================================================================
# Hexapod Navigation Pipeline – one‑shot setup & launch
# =============================================================================
# Prerequisites:
#   * ROS 2 Jazzy already installed at /opt/ros/jazzy
#   * Your workspace is at /home/daksh/spooder_ws
#   * You have sudo privileges (needed for apt install)
# =============================================================================

set -eo pipefail   # safety: exit on error, or pipefail

# -----------------------------------------------------------------------------
# 1. Source ROS 2 Jazzy
# -----------------------------------------------------------------------------
echo "[1/6] Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# -----------------------------------------------------------------------------
# 2. Install required system dependencies (needs sudo)
# -----------------------------------------------------------------------------
# echo "[2/6] Installing ROS 2 dependencies via apt (requires sudo)..."
# sudo apt-get update
# sudo apt-get install -y \
#     ros-jazzy-grid-map \
#     ros-jazzy-grid-map-ros \
#     ros-jazzy-depthai-ros \
#     ros-jazzy-nav2-bringup \
#     ros-jazzy-nav2-map-server

# -----------------------------------------------------------------------------
# 3. Prepare the workspace
# -----------------------------------------------------------------------------
WS_ROOT="/home/daksh/spooder_ws"
echo "[3/6] Preparing workspace at $WS_ROOT..."
cd "$WS_ROOT"

# Remove any previously cloned source overlays that might conflict with the
# Debian packages we just installed (they are not needed when we use the debs).
for dir in grid_map depthai-ros navigation2; do
    if [ -d "src/$dir" ]; then
        echo "   Removing conflicting source overlay: src/$dir"
        rm -rf "src/$dir"
    fi
done

# Clean build artefacts to avoid symlink‑install problems
echo "   Cleaning build/, install/, log/ directories..."
rm -rf build install log

# -----------------------------------------------------------------------------
# 4. Build the workspace (symlink install for fast iteration)
# -----------------------------------------------------------------------------
echo "[4/6] Building workspace with symlink install..."
colcon build --symlink-install

# -----------------------------------------------------------------------------
# 5. Source the overlay
# -----------------------------------------------------------------------------
echo "[5/6] Sourcing workspace overlay..."
source install/setup.bash

# -----------------------------------------------------------------------------
# 6. Launch the full pipeline
# -----------------------------------------------------------------------------
echo "[6/6] Launching hexapod navigation pipeline..."
ros2 launch hexapod_nav full_pipeline.launch.py