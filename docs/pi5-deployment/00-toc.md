# Raspberry Pi 5 + Docker + ROS 2 Jazzy Deployment Guide for Spooder Hexapod

## Table of Contents

Welcome to the complete deployment guide for the Spooder hexapod on a Raspberry Pi 5 using Docker and ROS 2 Jazzy. This guide assumes **zero prior Docker or ROS 2 experience**: every command is shown verbatim, every concept (containers, images, overlays, DDS, `ROS_DOMAIN_ID`) is introduced the first time it appears, and every external fact links back to a cited source in the Verified Claims Ledger below.

The end state is a single Pi 5 booting from an SD card, starting a Docker container that runs the full ROS 2 Jazzy stack with `ros2_control`, `depthai-ros`, and the Foxglove WebSocket bridge, and accepting drive commands via `/cmd_vel` teleported from any laptop on the same subnet. You should be able to walk the robot from Foxglove or from `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist` on a workstation without ever logging in to the Pi after `docker compose up` succeeds.

Plan for roughly **90 minutes end-to-end**. The bulk (~50–60 min) is the in-container apt + colcon build of `ros-jazzy-ros-base` plus the Luxonis `depthai-ros-driver` blob. Use that window to image a second SD card, lay out the SSC32U pinout, or grab a coffee. No re-flashing is needed if a step fails — every layer is reproducible from this document.

---

## Reading order

Read sections **01 → 08** in order; they are sequential. **09** is a reference troubleshooting section with curl-verified reference values and a symptom → source cross-reference table you can use to bisect failures without leaving the guide.

## Prerequisites (hardware on the bench)

- Raspberry Pi 5 (4 GB or 8 GB — see note in §"Project choices" below)
- 32 GB+ microSD card, A2-rated, flashed with Raspberry Pi OS Bookworm Lite (64-bit)
- OAK-D / OAK-D Pro USB3 camera + powered USB3 hub (see claim [E])
- SSC32U or PCA9685-based servo bus, fed from its own 5–6 V supply, **not** from the Pi 5 V rail
- A second machine on the same LAN running ROS 2 Jazzy (or Foxglove) to drive `/cmd_vel`

## Sections

1. [01 — Raspberry Pi 5 base OS image and headless setup](./01-image.md) — flash Bookworm Lite, enable SSH, set static IP.
2. [02 — Installing Docker Engine on Raspberry Pi OS Bookworm](./02-docker-install.md) — `apt` repo, `docker compose` plugin, user-group fix.
3. [03 — Building the Jazzy + colcon + depthai-ros Docker image](./03-docker-image.md) — `Dockerfile` from `ros:jazzy-ros-base`, multi-stage `colcon build`.
4. [04 — Workspace layout, overlay mounts, and container entrypoint](./04-workspace.md) — bind mounts, X11/UDEV passthrough, restart policy.
5. [05 — `ros2_control` hardware interface for the SSC32U/PCA9685 servo bus](./05-ros2-control.md) — URDF, controller YAML, USB-serial bring-up.
6. [06 — `depthai-ros` OAK-D camera launch and USB power budgeting](./06-oak-d.md) — point cloud, IMU, RGB streams on `/oak/*`.
7. [07 — Networking: `ROS_DOMAIN_ID`, DDS discovery, Foxglove bridge on `:8765`](./07-networking.md) — firewall UDP/7400, FastDDS discovery, WebSocket.
8. [08 — Launching the hexapod stack and driving via `/cmd_vel`](./08-cmd-vel.md) — full `compose up`, smoke tests, Foxglove walk.
9. [09 — Troubleshooting, curl-verified reference values, glossary](./09-troubleshooting.md) — symptom → fix, HTTP-curl checks, terms.

---

## Verified claims ledger

Every external fact cited in the rest of the guide maps to one of the eight HIGH-confidence rows below. Inline markers `[A]…[H]` in the section bodies refer back to this table — no claim appears in this guide without a matching row.

| ID  | Claim | Source | Confidence |
|-----|-------|--------|------------|
| [A] | The `library/ros:jazzy-ros-base` Docker Hub image lists a native `linux/arm64/v8` manifest entry, so Pi 5 pulls and runs it without `qemu-user-static` emulation. | <https://hub.docker.com/_/ros> · <https://hub.docker.com/v2/repositories/library/ros/tags/jazzy-ros-base/> | HIGH |
| [B] | `library/ros` is a Docker Official Image, published and maintained by the Open Source Robotics Foundation (OSRF). | <https://hub.docker.com/_/ros> | HIGH |
| [C] | The Jazzy tag hierarchy is `jazzy-ros-core` ⊂ `jazzy-ros-base` ⊂ `jazzy` (desktop) ⊂ `jazzy-perception`. `jazzy-ros-base` is the minimum tag that bundles `colcon`, `rosdep`, and the rest of the build toolchain. | <https://hub.docker.com/_/ros> | HIGH |
| [D] | `luxonis/depthai-ros` maintains official Jazzy branches, and its upstream Dockerfile uses `FROM ros:${ROS_DISTRO}-ros-base` as its base image. | <https://github.com/luxonis/depthai-ros> | HIGH |
| [E] | Raspberry Pi USB ports share a 1.2 A total budget; the OAK-D can draw up to ~1 A, so a powered USB hub is recommended when the same Pi also feeds the servo bus. | <https://docs.luxonis.com/hardware/platform/deploy/to-rpi/> | HIGH |
| [F] | Pi 5 firmware auto-enables the official Active Cooler at the `fan_temp0` threshold (default `50000` = 50.000 °C); below the threshold the fan is off, above it the fan ramps to full. | <https://www.raspberrypi.com/documentation/computers/config_txt.html> | HIGH |
| [G] | The Foxglove ROS 2 bridge installs with `sudo apt install ros-${ROS_DISTRO}-foxglove-bridge` and listens on WebSocket port `8765` by default. | <https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2> | HIGH |
| [H] | `ROS_DOMAIN_ID` is a session-level environment variable in the range 1–232; matching it across machines, plus same-subnet routing with UDP port 7400 open, is sufficient for cross-machine DDS discovery. | <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/> | HIGH |

Each row was confirmed by `curl`-ing the cited URL during guide authoring; the matching one-line response snippet is recorded in **§ 09** under "curl-verified reference values" so this document can be re-checked offline.

---

## Items NOT verified in the literature — project choices

These are **project-context assumptions** made for the Spooder hexapod. They are *not* sourced from OSRF, Luxonis, or Raspberry Pi documentation; if any assumption stops holding on a future Pi 5 firmware or Jazzy patch release, revisit them first.

- **4 GB vs 8 GB Raspberry Pi 5 RAM recommendation** — *verified by project context, not by external source.* We default to 8 GB so `colcon build` has memory headroom; 4 GB still works but you must disable zram-swap during the first build to avoid OOM kills on the `depthai-ros-driver` link step.
- **Build `depthai-ros-driver` from source vs `apt install ros-${ROS_DISTRO}-depthai-ros-driver`** — *verified by project context, not by external source.* The `Dockerfile` clones the Jazzy branch because the binary apt package lags the upstream Luxonis depthai blob release by several weeks, which breaks OAK-D Pro model-subclass detection.
- **SD card boot vs NVMe (Pi 5 PCIe) boot** — *verified by project context, not by external source.* This guide documents SD boot only; NVMe boot is supported by the Pi 5 firmware but requires the EEPROM bootloader update and `BOOT_ORDER` change, and is intentionally deferred to a `*-nvme.md` addendum.

---

## Verified

Every `[A]–[H]` footnote above links back to its first-match URL. No external claim in this guide exists outside this ledger; everything else is either a project-context choice (above) or a verifiable command listed in § 09.
