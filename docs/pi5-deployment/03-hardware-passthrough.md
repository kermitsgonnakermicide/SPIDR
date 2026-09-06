# Hardware Passthrough

## Why passthrough matters on Pi 5

ROS 2 Jazzy on a Raspberry Pi 5 running Raspbian bookworm requires both specific `/dev`
entries (so ros2_control and the OAK-D node can open hardware) and the right cgroup
allowances so the container can recreate device nodes on hotplug. The syntax for granting
a whole device class — not just one node — is `--device-cgroup-rule`, documented at
<https://docs.docker.com/reference/cli/docker/container/run/#device-cgroup-rule>. Missing
either half is the usual reason joints stay at zero or the OAK-D never enumerates.

## Canonical `docker run` flag block

```bash
docker run -d --name spooder \
  --restart unless-stopped \
  --network=host \
  --ipc=host \
  --privileged \
  --group-add dialout \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  --device-cgroup-rule 'c 188:* rwm' \
  --device /dev/bus/usb \
  --mount type=bind,source=/home/spooder/spooder_ws,target=/ws \
  -e ROS_DOMAIN_ID=42 \
  ros:jazzy-ros-base \
  ros2 launch spooder_bringup bringup.launch.py
```

Each line, annotated:

| Flag | Reason |
|------|--------|
| `--network=host` | DDS over wlan0 avoids NAT and multicast rewrite; shared-memory transport on `lo` works in host mode. |
| `--ipc=host` | Fast DDS uses POSIX SHM; default IPC namespace breaks `/dev/shm` based zero-copy between DDS participants. |
| `--privileged` | Pi 5 / bookworm needs full `cgroup` + `apparmor` drop for udev rebinding on servo hotplug. See caveat below. |
| `--group-add dialout` | Gives the in-container `spooder` uid (1000) the dialect group so it can `open()` `/dev/ttyUSB0` without a host-side `chmod 666`. |
| `--device /dev/ttyUSB0:/dev/ttyUSB0` | Passes the ST3215 serial bus through; ros2_control `serial_driver` opens this directly. |
| `--device-cgroup-rule 'c 188:* rwm'` | Major `188` covers `ttyUSB*`; lets the container recreate the node if the Feetech bus re-enumerates as a different minor (per [docker docs](https://docs.docker.com/reference/cli/docker/container/run/#device-cgroup-rule)). |
| `--device /dev/bus/usb` | OAK-D Lite needs raw libusb access via `depthai-core`, not the kernel UVC driver — node-level passthrough is insufficient. |
| `--mount type=bind,...` | Explicit `--mount` form per [docker bind-mount docs](https://docs.docker.com/engine/storage/bind-mounts/); unlike `--volume`, it does **not** auto-create the host dir, so a missing source fails fast at container start instead of silently creating an empty one. |

We use `--mount` rather than `-v` because `-v` silently `mkdir -p`s the host path. Pre-create
`/home/spooder/spooder_ws` on the host so the bind fails fast and visibly.

> **Caveat on `--privileged`:** The narrow alternative is `--cap-add=NET_ADMIN --device-cgroup-rule=...`
> plus an in-container `udevd`. We have seen that break under `systemd-udev-settle` retries
> with `permission denied` on `/dev/bus/usb/001/002`. For a single-robot production deploy,
> `--privileged` is the operationally robust choice; tighten later with `cap-drop` once
> the device set is frozen.

## RPi 5 USB power budget

Pi 5 shares 1.2 A across *all four* downstream USB-A ports by default. The OAK-D Lite
peaks near 1 A on boot, and the ST3215 bus draws another ~150 mA under stall. Both on
the Pi alone will brown out the rail and trip the polyfuse — joints will jitter, the
camera will reset mid-launch.

Two fixes; pick one, do not skip:

1. **Software:** add `usb_max_current_enable=1` to `/boot/firmware/config.txt` to lift
   the per-port current cap. Full reference:
   <https://www.raspberrypi.com/documentation/computers/config_txt.html>.
   Reboot for the OTP bit to take effect; verify with `vcgencmd get_config usb_max_current_enable`.
2. **Hardware (recommended for both OAK-D + ST3215):** use a powered USB-3 hub into one
   of the Pi's blue ports. The Luxonis deploy guide for Pi-class hardware is explicit:
   <https://docs.luxonis.com/hardware/platform/deploy/to-rpi/>. The Pi cannot source
   enough current for both peripherals, and `dwc2` current-limit events are only
   logged in `dmesg`, not surfaced to ROS.

## Deferred USB enumeration

`systemd-udev` on the Pi may take 2-3 s after boot to finish enumerating USB devices —
long enough that a `docker compose up` in the systemd unit races udev and the container
sees no `/dev/ttyUSB0`. Make the spooder container unit ordered against udev's settle
service, documented at
<https://github.com/systemd/systemd/blob/main/units/systemd-udev-settle.service>:

```ini
[Unit]
After=systemd-udev-settle.service
Wants=systemd-udev-settle.service
```

Inside the container, if a device is still missing after ~5 s (visible from
`ros2 control list_controllers` or `lsof /dev/ttyUSB0`), restart the container once
— `docker restart spooder` — after confirming the device is up on the host with
`ls -l /dev/ttyUSB0`. Do not loop-restart; figure out the rule in `/etc/udev/rules.d/`.

## `otg_mode` config.txt (when to ignore it)

The Pi 5's USB-C port has a legacy USB-2 controller attached to `dwc2`. The default
`otg_mode=0` keeps it in peripheral (device) mode; that only matters if someone wires
the OAK-D into the Pi 5's USB-C PD port. **Don't.** Use the blue USB3-A ports.

Standard passthrough via the blue ports does **not** touch `otg_mode` and does not need
this flag. If you ever see `otg_mode=1` in a tutorial, that tutorial is wiring the
camera into the wrong port. The config.txt reference is the same URL as above:
<https://www.raspberrypi.com/documentation/computers/config_txt.html>.
