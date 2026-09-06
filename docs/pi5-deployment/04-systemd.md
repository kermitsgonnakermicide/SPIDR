# systemd + Autostart

## Why a unit, not a `docker run` in `.bashrc`

A container that has to be started manually after every reboot is useless on a robot. Raspbian Bookworm ships with `systemd` as PID 1, and `systemd` is the canonical mechanism for turning a Docker container into a managed, auto-restarting, journaled service. We follow the well-known pattern in [Luzifer's "Docker + systemd" gist](https://gist.github.com/Luzifer/7c54c8b0b61da450d10258f0abd3c917) — one unit per container, `Type=simple` (because `docker run` does not fork), `Restart=always` so the controller comes back without intervention, and `Wants=network-online.target` so DDS discovery happens after the link is up rather than racing it. The alternative (`docker compose up -d` from a cron `@reboot`, or `if-up.d` hooks) either loses log aggregation, retry semantics, or both — none of which we want on a moving robot.

## The unit file

Drop this at `/etc/systemd/system/spooder.service`. Owned by `root`, mode `0644`.

```ini
[Unit]
Description=Spooder hexapod Docker container
After=docker.service network-online.target systemd-udev-settle.service
Wants=network-online.target

[Service]
Type=simple
Restart=always
RestartSec=5
User=spooder
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ExecStart=/usr/bin/docker run --rm --name spooder \
    --net=host \
    --ipc=host \
    --privileged \
    --device /dev/ttyUSB0 \
    --device /dev/bus/usb \
    --device /dev/dri \
    --group-add dialout \
    -v /home/spooder/spooder_ws:/ws \
    -v /home/spooder/hexapod_config.json:/home/spooder/hexapod_config.json:ro \
    spooder:jazzy-pi5 \
    ros2 launch spooder_robot robot_bringup.launch.py \
        calibration_json_path:=/home/spooder/hexapod_config.json \
        use_oakd:=true \
        use_sim_time:=false \
        usb_port:=/dev/ttyUSB0
ExecStop=/usr/bin/docker stop spooder
TimeoutStartSec=180

[Install]
WantedBy=multi-user.target
```

## Walkthrough of the load-bearing lines

Most of this mirrors §03 (`--net=host`, `--ipc=host`, `--privileged`, `--device /dev/bus/usb`, `--group-add dialout`); see that section for the per-flag rationale on hardware passthrough, the cgroup rule for major `188`, and the `usb_max_current_enable=1` power-budget fix. The systemd-specific lines are the ones below.

| Directive / flag | Reason |
| --- | --- |
| `After=docker.service` | `ExecStart` invokes the `docker` CLI; the unit will fail-early if Docker's own socket isn't up. |
| `After=network-online.target` and `Wants=network-online.target` | Fast DDS over `wlan0` needs the address present *before* the participant is built, or initial discovery silently completes with no remote peers. `Wants=` (not `Requires=`) makes the unit boot even if `network-online.target` fails — useful when the Pi starts on a tether with no DHCP, e.g. on the bench. The `network-online.target` wait behaviour is documented at <https://www.freedesktop.org/software/systemd/man/latest/systemd.network.html>. |
| `After=systemd-udev-settle.service` | udev may take 2–3 s after boot to give `/dev/ttyUSB0` its final symlink. Without this, a fast-booting Pi can race the daemon and the container exits with "no such file". |
| `Type=simple` | The PID systemd tracks is the `docker run` process. `docker run` does not fork — it execs into the container's PID 1 — so `simple` is correct. Do **not** change to `forking`; `dockerd` already has its own unit. The container-as-simple-service pattern is the same one in <https://gist.github.com/Luzifer/7c54c8b0b61da450d10258f0abd3c917>. |
| `Restart=always` | Even on a clean `exit 0` (which shouldn't happen, but does when a sibling process launches the unit manually and then types `Ctrl-C`), systemd re-runs `ExecStart`. For hardware bring-up where you want a `systemctl stop spooder` to stay stopped, swap to `Restart=on-failure`. |
| `RestartSec=5` | Five seconds is long enough that a `udev` rebind of `ttyUSB0` has time to settle, short enough that an OAK-D hot-unplug recovers before you reach the manual reset button. |
| `User=spooder` | The `spooder` user was created in §02. The `docker` daemon socket is group `docker`, and `spooder` is a member, so this works without `sudo`. Running the unit as root would also work but pollutes bind-mounted files on the host with `root` ownership. |
| `Environment=ROS_DOMAIN_ID=0` | `0` is fine for a single-robot deployment; pick a different ID 1–232 if you ever run two Spooders on the same subnet, or if a neighbour has an unrelated ROS 2 stack that floods UDP/7400. Range and semantics documented at <https://docs.ros.org/en/jazzy/Concepts/About-Domain-ID.html> and the DDS-discovery walkthrough at <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/>. |
| `Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Cyclone DDS (the ROS 2 fallback) does not always negotiate over `--net=host` cleanly on Pi 5 / Bookworm; Fast DDS is the Jazzy default and behaves. Cross-vendor comparison at <https://docs.ros.org/en/jazzy/Concepts/About-Different-Middleware-Vendors.html>. |
| `--name spooder` | Two reasons. First, `ExecStop` (next row) needs a stable name. Second, a `docker run` without `--name` lets Docker auto-generate `spooder_practical_galileo`-style names; a fat-fingered `docker run` from a different shell then collides on the image's `/dev/ttyUSB0`, and you spend a Saturday wondering why the controller "happened to" stop responding. |
| `ExecStop=/usr/bin/docker stop spooder` | Sends SIGTERM, then SIGKILL after 10 s. systemd waits for `docker stop` to exit, which means the dtors in `robot_bringup` (and the OAK-D's `depthai-core` device close) actually run. |
| `TimeoutStartSec=180` | First-boot `depthai-ros-driver` calibration can take 60–90 s while it re-flashes the OAK-D MyriadX bootloader; without this, systemd declares the unit failed at 90 s default and you see `Main process exited, code=exited, status=1/FAILURE` in `journalctl` even though the bringup was on track. The unit can still legitimately fail — watchdog is 180 s, and Fast DDS's `deadline`/`liveliness` will surface a real failure before the budget expires anyway. |
| `WantedBy=multi-user.target` | Required. Without `[Install]`, `systemctl enable spooder.service` is a no-op and the unit will not start on boot. `multi-user.target` is the right place: the graphical.target is not in scope for a headless Pi, and basic.target runs too early (it does not wait for `network-online.target`). |

## Variables that belong in `/etc/spooder.env`, not the unit

For a single-robot deployment, keeping `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` inside the unit is fine — they are committed-to-git, environment-invariant values. If you ever run more than one image variant on the same SD card, move them to an `EnvironmentFile=`:

```ini
# /etc/systemd/system/spooder.service, Service section
EnvironmentFile=/etc/spooder.env
```

```bash
# /etc/spooder.env  (mode 0640, root:spooder)
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`systemd` will then prepend those to the process environment, and you can edit `/etc/spooder.env` without `daemon-reload`. For a calibration field swap, also redirect the bind-mounted JSON path into the env file:

```bash
HEXAPOD_CONFIG=/home/spooder/hexapod_config.json
```

and reference it from the unit as `${HEXAPOD_CONFIG}`. systemd's environment-file spec is at <https://www.freedesktop.org/software/systemd/man/latest/systemd.exec.html#EnvironmentFile=>.

## Installation

```bash
sudo cp docs/pi5-deployment/spooder.service /etc/systemd/system/spooder.service
sudo chown root:root /etc/systemd/system/spooder.service
sudo chmod 0644      /etc/systemd/system/spooder.service
sudo systemctl daemon-reload
sudo systemctl enable spooder.service      # creates the multi-user.target.wants/ symlink
sudo systemctl start  spooder.service      # do not wait for the next reboot to validate
```

The `enable` step is the meaningful autostart step: it drops a symlink at `/etc/systemd/system/multi-user.target.wants/spooder.service` so the unit reaches `active (running)` after every reboot without an interactive shell.

## Verifying it works

Run these in order. The point is to confirm the boot graph: udev settled → docker up → network online → spooder active.

```bash
systemctl status spooder          # one-shot: should show "Active: active (running)" within ~10-30 s
journalctl -u spooder -f         # follow bringup logs; Ctrl-C when you have enough
journalctl -u spooder -b         # everything from this boot only
docker ps --filter name=spooder  # should show one container, "Up" for > 5 s
```

From a laptop on the same LAN (after `source /opt/ros/jazzy/setup.bash` and matching `ROS_DOMAIN_ID`):

```bash
ros2 node list                   # should include /spooder/oak, /spooder/spooder_controller, ...
ros2 topic list | grep cmd_vel   # /cmd_vel must be wired both ways
ros2 topic hz /oak/rgb/image_raw # nonzero -> OAK-D made it up inside the container
ros2 control list_controllers    # all listed `joint_trajectory_controller`s should be `active`
```

If `ros2 node list` returns only nodes from your laptop, the discovery problem is `ROS_DOMAIN_ID` mismatch *or* the firewall on either side is blocking UDP/7400 (see §07).

## Common failure modes

| Symptom (in `journalctl -u spooder -n 200`) | Likely cause | Fix |
| --- | --- | --- |
| `bind: address already in use` on `:11811` (Foxglove bridge) | A previous container survived a manual `docker run` (i.e. one started outside systemd). | `docker rm -f spooder` then `systemctl restart spooder`. After this, do not start ad-hoc `docker run` instances with `--name spooder` while the unit owns that name. |
| `Could not open serial port /dev/ttyUSB0: [Errno 2] No such file or directory` | `ttyUSB0` re-enumerated to `ttyUSB1` (a common Feetech ST3215 surprise), or udev had not settled. | Confirm `After=systemd-udev-settle.service` is in the unit. If `ls /dev/ttyUSB*` on the host shows `ttyUSB1`, add a udev rule (see §05) that pins the bus by serial, then restart. |
| `discovery: server returned error ...` cycled, then fast restarts | `RMW_IMPLEMENTATION` was not set (default `rmw_cyclonedds_cpp` in some Jazzy installs), and Cyclone over `--net=host` on a Pi 5 occasionally races the daemon socket. | Re-check the `Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp` line is uncommented in the unit, then `sudo systemctl daemon-reload && sudo systemctl restart spooder`. |
| `Main process exited, code=exited, status=1/FAILURE` within seconds, no `[INFO] [launch]: ...` lines | `ExecStart` itself errored. Most often a typo in a launch arg (`use_oakd:=yess`), or `/home/spooder/hexapod_config.json` missing on the host. | Re-run the `ros2 launch` line **manually** from inside the image (`docker run --rm -it --entrypoint /bin/bash spooder:jazzy-pi5`) to surface the real stderr. |
| `status=203/EXEC` immediately | The `docker` binary is not at `/usr/bin/docker` (custom install prefix), or `User=spooder` cannot find it on its `PATH`. | Confirm `/usr/bin/docker --version` as `spooder` on the host; if on a non-default prefix, hard-code the absolute path in `ExecStart`. Per <https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html>, missing or un-executable executables yield exactly `203/EXEC`. |
| Robot boots, runs, then dies 90 s in, every boot | OOM kill of the container. `colcon`-built `depthai-ros-driver` plus `ros2_control` plus `foxglove_bridge` peak at ~1.2 GB resident on Pi 5. | `dmesg | grep -i oom` will show `oom-kill` from the cgroup. Either enable zram-swap (`sudo systemctl enable zram-swap`) or upgrade to the 8 GB Pi 5 — see §00 "Items NOT verified in the literature" for the project choice. |

## Tuning notes

- **`RestartSec=5`** is a compromise: 5 s is long enough that udev rebinds settle, short enough that you don't walk to the robot before recovery. Bump to `15` if you see `systemd[1]: spooder.service: Scheduled restart job, restart counter is at N` incrementing more than ~3 times per hour — that's a real crashing bug, not transient noise.
- **`Type=simple` vs `Type=notify`.** `Type=notify` would let the unit signal "ready" via `sd_notify(0, "READY=1")` once Foxglove and DDS are both up, and systemd would refuse to mark the service active until then. `Type=simple` says "active immediately, trust the bringup to fail later if it must." Stay on `simple` until we wire `sd_notify` into `robot_bringup`'s launch lifecycle; the change is non-trivial in Python launch files and not worth it for a single-robot deployment.
- **Reload without restart.** Editing `/etc/systemd/system/spooder.service` does **not** require a container restart, only `sudo systemctl daemon-reload`. The next `systemctl restart spooder` (or the next reboot) picks up the change.
- **Hard-mode hardening (deferred).** For a single robot on a trusted home network, the unit above is sufficient. If you ever put Spooder on a shared or hostile network, add under `[Service]`:
  ```ini
  NoNewPrivileges=true
  ProtectControlGroups=true
  ProtectKernelTunables=true
  RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK
  ```
  …and then audit every `--device` you have; some of these flags break `--device /dev/bus/usb` until you also set `RestrictDeviceGroups=` correctly.
- **Pausing for hardware bring-up.** During bench work, set `Restart=on-failure` (not `always`) and use `systemctl start` / `stop spooder` from your own prompting. This avoids systemd silently respawning the container over a transient OAK-D disagreement, which is the most confusing possible signal during calibration.

The unit is now part of Raspbian's boot graph: power on → udev settled → docker daemon up → network-online → `spooder.service` started → bringup launched → DDS participants discovered. The next section (`05`) wires the `udev` rules that pin the Feetech bus and the OAK-D to stable device paths, so this whole sequence is idempotent across power-cycles.
