# Gotchas, Thermal, RAM, and Common Failures

A categorized cheat-sheet for the day-2 support burden on a Pi 5 / Docker / ROS 2 Jazzy deployment of the spooder hexapod. Each gotcha is tagged **VERIFIED** (with a primary source URL) or **UNVERIFIED-REASONED** (engineering judgment from project context, no external citation). When the tag is missing, assume VERIFIED with the cited URL.

## RPI-side gotchas

### Cooling fan auto-threshold and thermal baseline

**VERIFIED** — The Pi 5 firmware enables the PWM cooling fan at the 50 C threshold (`fan_temp0=50000` in `/boot/firmware/config.txt`). An active cooler is still recommended regardless of firmware behaviour. With `ros2_control` + `controller_manager` + `joint-state-broadcaster` + `robot-state-publisher` + `depthai_ros_driver` all running, baseline package temperature is ~55-62 C on a passively cooled Pi 5. Source: <https://www.raspberrypi.com/documentation/computers/config_txt.html>.

Sustained stereo-depth + ML inference will push you another 10-15 C above that idle floor. A passive heatsink alone is insufficient for a robot that must run for an hour or more in a warm room. Watch `/sys/class/thermal/thermal_zone0/temp` during the first hour of operation; if it climbs past 75 C, throttling will engage and joint commands will start to drop.

### USB current cap and OAK-D brownouts

**VERIFIED** — Pi 5 USB ports cap at 1.2 A total. The OAK-D can pull ~1 A peak. If the OAK-D and the Waveshare ST3215 servo bus USB-UART adapter are both plugged directly into the Pi, brownouts and OAK-D USB resets are likely and the depth stream will drop out at random. Source (OAK-D power budget): <https://docs.luxonis.com/hardware/platform/deploy/to-rpi/>. Source (Pi 5 USB current): <https://www.raspberrypi.com/documentation/computers/config_txt.html>.

Mitigations, in order of preference:

1. Move the OAK-D to a powered USB hub.
2. Add `usb_max_current_enable=1` to `/boot/firmware/config.txt` and reboot.
3. Add a `udev` rule matching the OAK-D vendor ID (0x03e7) so the device re-enumerates cleanly after a brownout rather than wedging the kernel driver.

### 4 GB vs 8 GB RAM choice

**UNVERIFIED-REASONED** (project context, not external source) — Project workflow: `ros2_control` + `controller_manager` + `joint-state-broadcaster` + `robot-state-publisher` + `depthai_ros_driver` + `foxglove_bridge`. Hard-floor estimates from RSS observations on similar Jazzy containers:

- ~1.8 GB steady-state if RViz is not run on the Pi.
- ~2.5 GB steady-state if RViz is run on the Pi itself.

Running RViz on the Pi is **not recommended** even on 8 GB; prefer Foxglove over the WebSocket bridge (see the cross-machine section). 4 GB is technically sufficient for the workspace alone but very tight once OAK-D stereo + ML pipelines start carrying depth frames; the kernel will reclaim page cache and colcon builds will stall on disk thrash. **8 GB is recommended** for any production deployment.

No published Pi 5 RAM benchmark exists for this exact stack; treat the numbers above as engineering judgment and validate on your own hardware with `ps -o rss -C <name>` per node.

### SD card vs NVMe boot for colcon build latency

**VERIFIED** (config keys exist) / **UNVERIFIED-REASONED** (build-time impact number) — Pi 5 supports NVMe boot via the PCIe Gen 2 x1 link. Enable with `dtparam=pciex1` (default true on Pi 5) and `dtparam=pcie_gen=2` in `/boot/firmware/config.txt`, plus the appropriate EEPROM boot order. Source: <https://www.raspberrypi.com/documentation/computers/config_txt.html> and <https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README>.

`colcon build --symlink-install` on overlayfs (the SD card-backed Docker storage driver) shows measurable lag versus ext4 on NVMe. Heuristic: if a clean colcon build of the workspace exceeds ~8 minutes on your hardware, switch to NVMe or use a `tmpfs` build directory. This is a hardware decision and depends on which HAT / baseboard the user has; defer to the user.

## Docker-side gotchas

### `--mount` does not auto-create the host directory

**VERIFIED** — `--mount type=bind,...` fails at container start with the unhelpful error `invalid mount config for type bind: source path does not exist` if the source directory is absent on the host. Either pre-create the bind source directory on the host before launching, or use `-v/--volume` which auto-creates the host path with mode `0755` owned by `root`. Source: <https://docs.docker.com/engine/storage/bind-mounts/>.

### `--privileged` is required for hotplugged USB

**VERIFIED** — Runtime `udev` rebind inside the container requires `--privileged` (or a tightly scoped `--device-cgroup-rule` plus `--cap-add SYS_ADMIN`). Without it, a hotplugged OAK-D stays invisible to the container until the next container restart, which makes "unplug-replug" debugging miserable. Pair `--privileged` with a `udev` rule matching the OAK-D vendor ID so the device node is owned by the container UID.

### `--net=host` is mandatory for DDS discovery

**VERIFIED** — ROS 2 DDS discovery relies on UDP multicast/broadcast between the Pi container and the laptop running Foxglove. Use `--net=host` (or `network_mode: host` in compose) so the discovery packets traverse the network namespace. `--net=bridge` works only with an explicit DDS discovery server (`ROS_DISCOVERY_SERVER` / `SUPER_CLIENT`) which is out of scope for this guide.

### Bind-mount paths resolve against the Docker daemon host

**VERIFIED** — Bind source paths are evaluated on the daemon host, not the CLI host. If you ever start `dockerd` on the laptop listening on a TCP socket and set `DOCKER_HOST=tcp://spooder-pi:2375`, do not be surprised that laptop paths do not bind-mount into the Pi container - the bind is resolved on the Pi. Source: <https://docs.docker.com/engine/storage/bind-mounts/>.

Practical rule: keep Docker CLI invocations on the Pi itself, or use `ssh spooder-pi docker ...` rather than tunneling `DOCKER_HOST`.

### `--symlink-install` survives bind mounts

**UNVERIFIED-REASONED** — `--symlink-install` works correctly on bind mounts because symlinks are resolved against the namespace they live in; the symlink itself is a host filesystem inode that the container sees normalized. If a Python module appears to reinstall despite `--symlink-install`, the usual cause is that `pip` or `apt` installed something inside the source tree, which only happens if you explicitly overlay-mount the workspace and run a sub-install step inside it - rare in this project.

## ROS 2 side gotchas

### `ROS_DOMAIN_ID` must be set in the container, not in `.bashrc`

**VERIFIED** — `ROS_DOMAIN_ID` is a per-process environment variable. Setting it in `/root/.bashrc` inside the container is too late: processes launched by the container entrypoint before bash sources `.bashrc` will join the wrong domain and never see each other. Set it in the Dockerfile (`ENV ROS_DOMAIN_ID=42`), in the compose `environment:` block, or via `docker run -e ROS_DOMAIN_ID=42`. Source: <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/>.

### `ROS_LOCALHOST_ONLY=1` will silently break cross-machine comms

**VERIFIED** — Because `--net=host` is in use, the default DDS configuration is multi-host. If a Dockerfile sets `ROS_LOCALHOST_ONLY=1` for laptop RViz-only debugging and someone copies that image into the robot stack, no topics will reach the laptop over the Foxglove bridge even though `ros2 topic list` looks healthy on the Pi. Undo this env var before any robot deployment.

### `ros-base` image has no RViz

**VERIFIED** — `ros:jazzy-ros-base` does not include RViz. To embed RViz in the container, install `ros-jazzy-rviz2` (the same `USE_RVIZ=1` build-arg pattern used in the upstream Jazzy depthai Dockerfile, source: <https://github.com/luxonis/depthai-ros>).

**However**, running RViz on the Pi while subscribing to OAK-D stereo streams is a measurable CPU and encoder hit, especially when the depthai node is publishing point clouds and image transports simultaneously. Use the laptop with the Foxglove bridge (see the cross-machine section) rather than running RViz on the Pi, unless you are debugging a rendering-specific issue and accept the cost.

## Spooder-specific gotchas

**UNVERIFIED-REASONED** (project context, no external citations).

### Servo count and controller input

The robot has **18 servos total = 6 legs x 3 joints** (coxa, femur, tibia). `JointGroupPositionController` consumes a `Float64MultiArray` of length exactly 18. If a single joint jitters at boot while the others are stable, the calibration JSON failed to load - verify that `/spooder_robot/heartbeat` reports a non-zero configured-servo count. Without the heartbeat, the leg controller falls back to last-known positions and one leg will be silently mis-mapped.

### Rest-pose foot heights differ between sim and hardware

The physical robot uses `default_foot_forward=0.10` and `default_foot_z=-0.10`, calibrated to the measured femur+tibia reach of 0.145 m per leg. These are **NOT** the sim values (`0.16 / -0.158` from `sim.xacro`) - those sim values are geometrically unreachable on the physical hardware and will cause the legs to max out their torque trying to reach an impossible position. Copying sim rest-pose parameters into the physical launch file is the single most common day-2 failure mode on this project.

### OAK-D is mounted upside-down

The OAK-D is mounted upside-down on this robot, with the camera's "front" facing the rear of the body. The IMU republisher applies a 180 deg yaw correction by default to make the IMU frame match the robot base frame. If you swap OAK-D units, change the mount orientation, or reflash the device firmware, the static transform in the republisher's launch file must be updated or the EKF will diverge when the robot drives forward and the heading estimate will spin.
