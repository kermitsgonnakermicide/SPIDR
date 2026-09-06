# Section 2: Dockerfile for the Pi 5 / Jazzy stack

This section builds the spooder hexapod container on top of the upstream
`ros:jazzy-ros-base` image ([Docker Hub: ros](https://hub.docker.com/_/ros)).
The base image is multi-arch — `linux/amd64` and `linux/arm64v8` — so the same
Dockerfile builds natively on a desktop for cross-debugging and natively on the
Raspberry Pi 5 for deployment.

The image layers fall into four groups:

1. **Apt system deps** — `libserial-dev` (ST3215 servos ride USB serial on
   `/dev/ttyUSB*`; libserial gives a clean C++ wrapper), `libeigen3-dev`
   (transform math for leg kinematics and IMU fusion).
2. **`ros-jazzy-*` packages** — `ros2-control`, `ros2-controllers`,
   `robot-state-publisher`, `controller-manager`, `joint-state-broadcaster`,
   `joint-trajectory-controller`, `xacro`, `depthai-ros-driver`
   ([Luxonis depthai-ros Jazzy branch](https://github.com/luxonis/depthai-ros)),
   `foxglove-bridge`
   ([Foxglove ROS 2 docs](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)),
   `rmw-fastrtps-cpp`, plus `colcon` and the Python build helpers.
3. **Non-root user** — follows the
   [osrf/docker_images](https://github.com/osrf/docker_images) dev-container
   convention: a `spooder` user at uid 1000 / gid 1000, added to the `dialout`
   group so it can open `/dev/ttyUSB*` for the ST3215 serial bus, plus
   passwordless `sudo` for diagnostic commands.
4. **Workspace bind-mount self-build** — the source tree lives on the host and
   is bind-mounted at `/ws`; the first `colcon build` runs *inside* the
   container, not at image build time. This keeps the image small and lets the
   user iterate on Python nodes without rebuilding the image.

## The Dockerfile

```dockerfile
# syntax=docker/dockerfile:1.7
# spooder hexapod — Pi 5 / Jazzy base image.
# Layout follows osrf/docker_images conventions: ARG USERNAME block plus a
# non-root user in the dialout group for /dev/ttyUSB* serial access.

# ---- 1. Base image ----------------------------------------------------------
# Multi-arch base; linux/arm64v8 verified for Raspberry Pi 5.
FROM ros:jazzy-ros-base

# ---- 2. Build-time arguments (osrf/docker_images pattern) -------------------
ARG USERNAME=spooder
ARG USER_UID=1000
ARG USER_GID=1000

# ---- 3. Apt layer: system deps + ROS packages -------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libserial-dev \
      libeigen3-dev \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-xacro \
      ros-jazzy-depthai-ros-driver \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-rmw-fastrtps-cpp \
      python3-colcon-common-extensions \
      python3-pip \
      python3-numpy \
      python3-yaml \
 && rm -rf /var/lib/apt/lists/*

# ---- 4. Non-root user with dialout group for serial bus ---------------------
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
 && usermod -aG dialout,sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# ---- 5. Workspace mount point + ownership -----------------------------------
RUN mkdir -p /ws && chown ${USERNAME}:${USERNAME} /ws
WORKDIR /ws
USER ${USERNAME}

# Default entrypoint sources the Jazzy overlay for the shell.
CMD ["/bin/bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec bash"]
```

Notes on layer choices:

- **`ros-jazzy-depthai-ros-driver`** ships as a prebuilt `.deb` for `arm64`,
  so no from-source build of `depthai_core` is required on the Pi 5. Upstream
  Jazzy support lives in the `ros2-devel` branch
  ([luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)).
- **`ros-jazzy-foxglove-bridge`** is the apt name published by Foxglove for
  Jazzy ([Foxglove — Connecting to ROS 2](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)),
  avoiding a hand-pinned Studio-version mismatch.
- **`dialout` group membership** is what lets the container open
  `/dev/ttyUSB0`-style devices passed through with `--device` or
  `--privileged`; without it the ST3215 hardware interface fails with
  `EACCES`.

## Build the image

From the directory that contains the `Dockerfile`:

```bash
docker build -t spooder:jazzy-pi5 .
```

The tag `spooder:jazzy-pi5` is referenced by the `docker run` commands in the
later sections.

## First build *inside* the container

The workspace source tree is bind-mounted from the host (handled by the run
command in Section 3), so the image itself does not contain any of the
`spooder_*` packages. The first time you enter the container you build the
workspace once:

```bash
# Inside the running container
source /opt/ros/jazzy/setup.bash
cd /ws

colcon build --symlink-install \
  --packages-select spooder_robot spooder_hardware spooder_description
```

`--symlink-install` overlays each package's install tree with symlinks back
into the source tree. **For Python nodes** this means edits to `.py` files on
the host take effect on the next launch with no rebuild. **C++ packages
(`spooder_hardware`) still need a full `colcon build`** after changes to
`.cpp`, `.hpp`, or `CMakeLists.txt`.

Subsequent rebuilds after C++ edits:

```bash
colcon build --symlink-install --packages-select spooder_hardware
```

## References

- [ros:jazzy-ros-base on Docker Hub](https://hub.docker.com/_/ros) — official
  upstream base image, multi-arch.
- [osrf/docker_images](https://github.com/osrf/docker_images) — reference
  layout for the `ARG USERNAME` and non-root-user blocks used here.
- [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros) — Jazzy
  (`ros2-devel`) branch of the OAK-D ROS 2 driver; the prebuilt `.deb` is
  what `ros-jazzy-depthai-ros-driver` pulls in.
- [Foxglove — Connecting to ROS 2](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2)
  — apt-package name `ros-jazzy-foxglove-bridge` and bridge defaults.
