# OAK-D Lite Setup Guide

## Hardware Requirements

- OAK-D Lite camera
- USB 3.0 port (required for full bandwidth)
- Ubuntu 24.04 with ROS 2 Jazzy installed

## Software Dependencies

```bash
# Install ROS 2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install depthai packages
sudo apt install ros-jazzy-depthai-ros-driver \
                 ros-jazzy-depthai-ros-msgs \
                 ros-jazzy-depthai-bridge

# Verify installation
ros2 pkg list | grep depthai
```

## USB Permissions

The OAK-D Lite requires USB 3.0 access. Run once:

```bash
sudo ./scripts/setup_oakd_permissions.sh
```

This creates a udev rule that allows non-root users to access the camera.

**You may need to log out and log back in for group changes to take effect.**

## Verify Detection

```bash
# Check USB
lsusb | grep 03e7

# Should show:
# Bus 001 Device 012: ID 03e7:2485 Intel Movidius MyriadX

# Launch driver
ros2 launch spooder_perception oakd_driver.launch.py

# Check topics
ros2 topic list | grep oak_d
```

## Topic Interface

After launching, you should see:

| Topic | Type | Description |
|-------|------|-------------|
| `/oak_d/points` | PointCloud2 | Depth point cloud |
| `/oak_d/stereo/image_raw` | Image | Depth image |
| `/oak_d/rgb/image_raw` | Image | RGB image |
| `/oak_d/imu` | Imu | IMU data |
| `/oak_d/left/camera_info` | CameraInfo | Left camera intrinsics |
| `/oak_d/right/camera_info` | CameraInfo | Right camera intrinsics |
| `/oak_d/rgb/camera_info` | CameraInfo | RGB camera intrinsics |

## TF Tree

```
spooder/base_link
  └─ oak_d_base_frame
       ├─ oak_d_left_camera
       ├─ oak_d_right_camera
       ├─ oak_d_rgb_camera
       └─ oak_d_imu
```

## Calibration

The OAK-D Lite comes with factory calibration. For most experiments, factory calibration is sufficient.

### Custom Calibration

For high-precision applications:

1. Print a calibration checkerboard (7×9 squares recommended)
2. Clone the calibration tool:
   ```bash
   git clone https://github.com/luxonis/depthai-calibration.git
   ```
3. Run the calibration tool:
   ```bash
   cd depthai-calibration
   python3 calibrate.py --board <your_board_type> --rows 7 --cols 9
   ```
4. The calibration is stored on the device flash

## Troubleshooting

### Camera Not Detected

1. Check USB connection:
   ```bash
   lsusb | grep 03e7
   ```
2. Try a different USB 3.0 port
3. Check USB permissions:
   ```bash
   ls -la /dev/bus/usb/*/003e7*  # Should be rw-rw-rw-
   ```
4. Re-run permissions script:
   ```bash
   sudo ./scripts/setup_oakd_permissions.sh
   ```

### Low Framerate

1. Ensure USB 3.0 is used (check `lsusb -t`):
   ```
   5000M = USB 3.0 ✓
   480M = USB 2.0 (too slow)
   ```
2. Reduce resolution in config:
   ```yaml
   stereo:
     i_width: 480
     i_height: 270
   ```

### No Point Cloud

1. Check depth output:
   ```bash
   ros2 topic hz /oak_d/stereo/image_raw
   ```
2. Verify depth preset:
   ```yaml
   stereo:
     i_depth_preset: 'HIGH_ACCURACY'
     i_subpixel: true
     i_lr_check: true
   ```

### IMU Not Publishing

```bash
# Check IMU topic
ros2 topic hz /oak_d/imu

# Verify IMU is enabled
pipeline_gen:
  i_enable_imu: true
```

## Mounting the Camera

The OAK-D Lite should be mounted on the robot with:
- Lens facing forward
- Cable facing backward (toward cable routing)
- Firmly attached to minimize vibration

Recommended mounting position on Spooder:
- On the front of the body
- 5-10 cm above ground plane
- Slight downward tilt (10-15°) to see terrain ahead
