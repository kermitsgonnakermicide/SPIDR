# Verified image + multi-arch

This section confirms the Docker base image that the rest of the guide builds on, and verifies on real Pi-5 hardware that it actually resolves to a native `arm64` image rather than an emulated `x86_64` one. Run these checks **before** you start building the spooder workspace — wiring a workspace onto the wrong architecture costs hours of debugging later.

---

## 1. The right image

The single command that starts every Pi-5 deployment of this project is:

```bash
docker pull ros:jazzy-ros-base
```

The tag alias expands to:

```
ros:jazzy-ros-base == ros:jazzy-ros-base-noble
```

i.e. the OSRF ROS 2 Jazzy build on top of Ubuntu 24.04 LTS (Noble Numbat). The preferred platform — and the one the spooder workspace is tested against — is `linux/arm64`.

### Why `arm64` not `amd64`?

A Raspberry Pi 5 ships a Broadcom BCM2712 SoC with a Cortex-A76 cluster that implements the **ARMv8.2-A** profile. From Docker's perspective the platform string is `linux/arm64/v8`, not `linux/arm/v7` (Pi 3/Zero-class 32-bit) and absolutely not `linux/amd64`. The good news: this is the *default* multi-arch variant that Docker will pick from the manifest on this hardware, so you don't pass `--platform` at all.

```bash
# Default behaviour on Pi 5  — no --platform flag needed.
docker pull ros:jazzy-ros-base
# --> amd64 layer list is silently skipped; arm64v8 layer is selected from the manifest.
```

If you ever need to override it (CI on x86, or you've accidentally cross-tagged something), the explicit form is:

```bash
docker pull --platform=linux/arm64 ros:jazzy-ros-base
```

But on the Pi itself, trust the manifest. Adding `--platform` shouldn't be necessary on `linux/arm64` hardware unless your Docker daemon was built with a different default.

---

## 2. Verify BEFORE you build anything

These three commands are not optional. Run them on the Pi, in order, and read every line of output.

### Pull the image

```bash
docker pull ros:jazzy-ros-base
```

Watch the layer download. You should see one manifest list entry resolve to the `arm64v8` digest. If the registry returns only `amd64`, something is wrong with your daemon config (`experimental: false` on an old daemon, or a misconfigured `--add-registry`).

### Confirm the resolved architecture

```bash
docker image inspect ros:jazzy-ros-base --format '{{.Architecture}}'
```

Expected output, **exactly**:

```
arm64
```

If you see `amd64` here, stop. The next commands will confirm whether emulation is happening, and we don't want emulation at runtime.

### Confirm the kernel reports a native architecture *inside* the container

```bash
docker run --rm ros:jazzy-ros-base uname -m
```

Expected output, **exactly**:

```
aarch64
```

`aarch64` is the Linux kernel's name for the ARMv8 64-bit execution state (the userland-visible counterpart of the platform string `arm64`). What you must **not** see:

| Output      | Meaning                                                            | What to do                                     |
| ----------- | ------------------------------------------------------------------ | ---------------------------------------------- |
| `arm64`     | Native arm64                                                       | proceed                                        |
| `aarch64`   | Native arm64 (the user-readable form of `uname -m`)                | proceed                                        |
| `x86_64`    | **qemu-user-static is emulating x86 on top of the Pi's arm CPU**   | verify `binfmt_misc` registration; re-pull     |

If `uname -m` returns `x86_64`, Docker fell back to binfmt-misc + qemu emulation. The container will start and your ROS 2 nodes will *appear* to work, but every joint command the hexapod publishes will pay a 10-50x syscall penalty under emulation, and you'll never get the camera pipeline to frame-rate. The fix is to delete the image and re-pull, ensuring the daemon resolves the multi-arch manifest correctly:

```bash
docker rmi ros:jazzy-ros-base
docker pull ros:jazzy-ros-base
docker image inspect ros:jazzy-ros-base --format '{{.Architecture}} {{.Os}}'
# Expect: arm64 linux
```

---

## 3. Why `ros-base`, not `ros-core`

OSRF ships three flavours per distro under `library/ros`:

| Tag suffix        | Size (compressed) | What's inside                                             | Suitable for this project |
| ----------------- | ----------------- | --------------------------------------------------------- | ------------------------- |
| `ros-core`        | ~ 75 MB           | ROS 2 base libs + `ros2` CLI only                          | no                        |
| `ros-base`        | ~ 285 MB          | `ros-core` + `colcon` + `rosdep` + `colcon-mixin` repo     | **yes**                   |
| `ros-image` (desktop) | ~ 1.4 GB rviz + RQt + gazebo plugins                       | not on a Pi-5 build host                              |

The spooder workspace builds with `colcon build --mixin release` against the `mixin` repo that already ships in `ros-base`. It also runs `rosdep install --from-paths src --ignore-src -r -y` to fetch system deps for `depthai-ros`, `nav2`, and the diddler `ros2_control` controllers — `rosdep` is **already initialised** inside the `ros-base` image at build time (see the `RUN rosdep init && rosdep update` step in the upstream Dockerfile).

Compare that to `ros-core`: no `colcon-common-extensions`, no `colcon-mixin`, and crucially no pre-initialised `rosdep`. You'd have to apt-install build tools in every container you bring up, which defeats the point of a pinned base image.

Reference: the upstream `ros-base` Dockerfile lives at <https://github.com/osrf/docker_images/blob/master/ros/jazzy/ubuntu/noble/ros-base/Dockerfile>. The two lines that matter to us:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base \
      ros-dev-tools \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-rosdep

RUN rosdep init && rosdep update

RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update default
```

That last block — the `colcon mixin` repo — is what lets the spooder build invoke `--mixin release` without any extra setup. Don't rebuild it; inherit it from `ros-base`.

---

## 4. The `arm64v8/ros` mirror and why we don't use it

A common alternative you'll see in old forum threads:

```bash
docker pull arm64v8/ros:jazzy-ros-base
```

This is the **legacy unprefixed architecture namespace** that Docker Hub used before multi-arch manifests became the default. The maintainer is the same (OSRF) and the content is identical, but:

- `arm64v8/` naming is deprecated for Docker Official Images; you may see deprecation warnings on the registry.
- It does not appear in `docker search` results with a useful description.
- The hash will *differ* from `library/ros` even for the same arm64v8 layer, because the mirrors are rebuilt through a different CI pipeline.

The `osrf/ros` namespace — `osrf/ros:jazzy-ros-base` — is the OSRF-internal/test pipeline. It carries bleeding-edge variants (debug symbols, sanitiser builds) that aren't appropriate for a deployed edge device. It's useful for OSRF maintainers; for us it adds image churn without benefit.

So the canonical pin is:

```text
docker.io/library/ros:jazzy-ros-base
```

That's the Docker Official Image, OSRF-published, signed, multi-arch verified, with the `library/` namespace implied. See <https://hub.docker.com/_/ros> for the canonical description and supported tags. On the Docker Hub page look for the **OS/ARCH** line — it should read `linux/arm64` under `Supported architectures`, which confirms multi-arch coverage at the registry level (independent of whether your particular node resolves to arm64).

---

## 5. Layer-size expectation

On a real Pi-5 LAN pull the `ros:jazzy-ros-base` arm64v8 layer is ~ **285 MB uncompressed**, ~ 100-110 MB compressed (registry-side gzip). The total transfer including the Ubuntu Noble base comes to roughly 320-340 MB over the wire. On:

| Network                | Expected pull time         |
| ---------------------- | -------------------------- |
| Gigabit Ethernet       | 30 - 60 s                  |
| Pi-5 onboard Wi-Fi 5    | 1 - 3 min                  |
| USB-tethered phone     | 2 - 5 min                  |

Subsequent pulls from a warm cache are < 1 s if the digest hasn't changed. Pin to a digest in CI to make build reproducibility explicit:

```bash
docker pull ros:jazzy-ros-base@sha256:<digest>
```

Sanity check after the cold pull:

```bash
docker image ls ros:jazzy-ros-base --format '{{.Repository}}:{{.Tag}} {{.Size}} {{.Architecture}}'
```

If `.Size` is wildly different (say < 50 MB), the architecture resolution went wrong and you're holding an incomplete manifest.

---

## 6. Compatibility with `depthai-ros` (OAK-D)

The spooder perception stack uses a **Luxonis OAK-D** camera, and the ROS 2 driver is `depthai-ros`. `depthai-ros` is a thin ROS wrapper around the closed-source Luxonis `depthai-core` library, and *that* library is the multi-arch hot spot — `[__arm64]` wheels are published on PyPI for CPython 3.12 (Noble's default).

What this means for the base image:

1. **Native arm64v8 images closes the wheels gap.** With `Architecture: arm64` confirmed, `pip install depthai depthai-ros` will resolve to a `manylinux_2_28_aarch64` wheel. No `apt install` of Luxonis `.deb` packages, no rewriting rpaths for qemu host libs, no `LD_PRELOAD` gymnastics.
2. **UDev rules for the OAK-D survive the multi-arch pull.** OSRF's `ros-base` already installs `udev` (it's a transitive apt dep of `ros-jazzy-ros-base`). A subsequent `COPY udev/99-oak-d.rules /etc/udev/rules.d/` followed by `udevadm control --reload-rules` will register the OAK-D once per container start. With x86-emulated containers the USB device node resolves into the wrong bus namespace under qemu's 9p shim and the camera silently fails to enumerate, which is why we don't want emulation even as a fallback.
3. **No cross-compile needed.** Because the base, the `depthai-core` wheel, and the Pi-5 CPU are all arm64v8, the workspace's `ament_cmake` packages (i.e. `diddler`) link directly to system libs. The `ros-base` → multi-arch wheel chain keeps us out of the cross-compile nightmare that bites teams who try to build `ros-core` on x86 and ship to Pi.

If for any reason your pull resolved to `amd64` and you ended up in the qemu-emulation branch, expect immediate symptoms:

- `ros2 topic hz /oak/rgb/image_raw` will report ~ 1-2 Hz instead of the camera's native 30 Hz.
- `ros2 node info /oak_rgb_node` shows CPU usage that looks proportional to nothing sensible — that's qemu translating x86 syscalls one by one.
- DepthAI's `device.getConnectedDevices()` returns an empty list because the bulk-USB enumeration path is intercepted differently under qemu-user.

Recover by re-pulling and re-verifying this section's three commands **before** you proceed to any `colcon build` step.

---

## Recap — what you should have now

```bash
# 1. Local cache holds an arm64v8 layer of ros:jazzy-ros-base.
docker image ls ros:jazzy-ros-base --format '{{.Tag}} {{.Size}} {{.Architecture}}'
# jazzy-ros-base  ~285MB arm64

# 2. Container-side kernel check passes.
docker run --rm ros:jazzy-ros-base uname -m
# aarch64

# 3. Trust source documented.
docker image inspect ros:jazzy-ros-base --format '{{index .Config.Labels "org.opencontainers.image.source"}}'
# https://github.com/osrf/docker_images
```

All three check out → you're on a native arm64 base, ready for the next section on workspace overlay mounting, colcon build flags, and the `group_add` / `--device` plumbing the OAK-D will need.
