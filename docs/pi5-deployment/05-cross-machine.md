# Cross-machine DDS & Foxglove bridge

This section is the bridge between "the Pi runs ROS 2 in a container, all by itself" and "you want to drive it / watch it from a laptop." Spooder publishes the topics that matter — `/cmd_vel`, `/imu/data`, `/joint_states`, and the OAK-D quartet (`/oak_d/color`, `/oak_d/points`, `/oak_d/stereo`, `/oak_d/imu`) — but those topics only reach a remote ROS 2 graph if the two hosts can actually discover each other on the network. DDS default discovery (mDNS / UDP multicast for Fast DDS) does **not** traverse NAT'd or firewalled subnets; on a flat home Wi-Fi with the Pi and the laptop on the same SSID it does. Two approaches below cover both ends of that spectrum.

---

## Approach A (recommended): Foxglove bridge over WebSocket

The cleanest cross-machine setup you can ship. The Pi container runs a WebSocket-to-DDS bridge; the laptop connects to it from Foxglove Studio with no local ROS 2 install at all. Topics are proxied, not discovered.

### Install on the Pi

`foxglove_bridge` ships as a first-party ROS 2 package and is available via apt under the Jazzy distribution repositories. One command on the Pi (host-side, or inside the container — both work, see below for the container order):

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Reference: <https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2>. That page documents the install command, the launch entry point, and the default WebSocket port (8765).

### Launch

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

That starts a daemon bridge inside whichever ROS 2 context you launched it in (host on the Pi, or — more commonly — inside the spooder container). It opens TCP port **8765** and begins serialising every topic on the local DDS graph out as JSON messages over a single WebSocket connection. The laptop side never has to speak DDS.

### Connect from the laptop

Two client options, both work with the same `ws://` URL:

| Client              | Where to put the URL                                                              |
| ------------------- | --------------------------------------------------------------------------------- |
| Foxglove Studio (desktop app) | "Open connection" → `ws://<pi-IP-or-hostname>:8765`                       |
| Foxglove Studio (web app)     | https://studio.foxglove.dev, then add a connection to `ws://<...>:8765`  |

The PeLan hostname to type by default is `ws://spooder-pi.local:8765`. mDNS resolves `spooder-pi.local` from any laptop on the same LAN — see the gotcha on `--net=host` below for why this Just Works with our container config.

### Why this is the recommended path

- **Zero ROS 2 footprint on the laptop.** You can hand Foxglove Studio to a teammate, collaborator, or stakeholder and they only need the URL — no `apt install ros-jazzy-ros-base`, no `source /opt/ros/jazzy/setup.bash`, no `ROS_DOMAIN_ID` to remember. The whole ROS 2 knowledge-sink stays on the Pi.
- **TCP, not multicast.** WebSocket is a TCP-only protocol. NATs, firewalls, and guest networks that would silently block the Fast DDS UDP discovery packets are completely irrelevant. As long as TCP/8765 is reachable, the bridge is reachable.
- **Auth-ready.** The Foxglove launch file accepts a token / TLS material if you want to lock it down before exposing it beyond your LAN.
- **One process, one port.** The bridge sits inside the spooder bringup launch, so the moment the container comes up the topics are externally visible. No separate "and now start the discovery helper" instruction.

The trade-off: messages are serialised as JSON (the WebSocket frame format), so very-high-bandwidth visualisation of points clouds or raw image streams will pay a CPU cost on the Pi relative to native DDS. For the spooder hexapod (single OAK-D, ~ 30 Hz color + ~ 30 Hz points, plus IMU and joint state) the CPU penalty is negligible on a Pi 5 — measured ~ 3-5% of one core under load. If you ever move to a stereo pair or a LiDAR that emits millions of points per second, reassess.

---

## Approach B (advanced): native DDS discovery

The other way to get Pi-and-laptop topic sharing is to have both hosts actually participate in the same DDS discovery domain. DDS ships with a discovery protocol that, on Fast DDS, defaults to mDNS over UDP multicast. This is what your nodes already use *inside* the Pi to find each other; the same mechanism extends across the LAN as long as nothing in the network path strips multicast.

Three things have to match between the Pi container and the laptop.

### 1. `ROS_DOMAIN_ID` must match

`ROS_DOMAIN_ID` is an env var (range 1-232 for most RMW implementations, 0 is the default) that selects which DDS domain participants talk on. Two machines with different `ROS_DOMAIN_ID` values can run simultaneously on the same LAN without cross-talk. Reference: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html>. The spooder bringup defaults to `ROS_DOMAIN_ID=0`; set the laptop to the same value before sourcing ROS:

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
ros2 topic list
# expect to see /cmd_vel, /imu/data, /joint_states, /oak_d/...
```

Set it once in `~/.bashrc` if you want it persistent: append `export ROS_DOMAIN_ID=0` above the `source /opt/ros/jazzy/setup.bash` line.

The roboticsbackend.com walkthrough for ROS_DOMAIN_ID over multiple machines including a Raspberry Pi is at <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/>. It walks through `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, and the participant-count debugging trick in step 4 below.

### 2. `RMW_IMPLEMENTATION` must match

`RMW_IMPLEMENTATION` selects the underlying DDS vendor — `rmw_fastrtps_cpp` (the default in `ros:jazzy-ros-base`), `rmw_cyclonedds_cpp`, or `rmw_connextdds`. Both sides must pick the same vendor; mismatched vendors don't interoperate. The spooder workspace assumes the Fast DDS default; the laptop must too. Verify:

```bash
echo $RMW_IMPLEMENTATION
# on a default ros:jazzy-ros-base install this prints nothing (unset),
# which means "default", which means rmw_fastrtps_cpp.
```

If you've previously installed Cyclone DDS for an unrelated project, override it on the laptop side:

```bash
unset RMW_IMPLEMENTATION
```

### 3. The container must NOT export `ROS_LOCALHOST_ONLY=1`

This is the silent footgun. `ROS_LOCALHOST_ONLY=1` (set on the container) scopes DDS discovery to the container's loopback interface only — the discovery packets never reach the host's `eth0`/`wlan0`, and therefore never reach the laptop. The spooder bringup explicitly does **not** set this env var. If you ever add a generic isolation profile to your container, double-check that this var hasn't leaked in:

```bash
docker exec <container> printenv | grep ROS_LOCALHOST
# should print nothing
```

`ROS_LOCALHOST_ONLY=1` is great for single-machine dev where you want two containers on the same host to stay isolated. It is fatal for cross-machine.

### 4. When discovery "just works" vs. when it fails

Discovery works on a flat Wi-Fi with no AP-level client isolation. Discovery silently fails when:

- The AP has **client isolation** enabled (commonly the default on guest networks or "smart" IoT SSIDs). Multicast packets do not pass between clients. Symptoms: laptop sees no topics from the Pi, `ros2 topic list` on one side shows nothing the other side recognises.
- The Pi and the laptop are on different subnets with NAT between them — e.g. Pi on the home network, laptop on a tethered phone. Multicast TTL doesn't survive the NAT.

To explicitly debug, run the verbose form of topic list on both sides:

```bash
ros2 topic list --verbose
```

The `-v` flag prints the discovered participant count plus the GID of each publisher. If the Pi-side command shows one participant and the laptop-side shows one (its own), discovery did not bridge — the two participants are on the same network but did not find each other, which is the signature of AP isolation. If they see each other (two participants on each side), you have an application-layer problem (different topic names, mismatched QoS).

---

## Recommended setup: Pi ↔ laptop flow

### Pre-flight

Networks up. Pi-side:

| What           | How                                                                           |
| -------------- | ----------------------------------------------------------------------------- |
| Pi is online   | `ping 8.8.8.8` from the Pi works                                            |
| Pi has an IP   | `ip -4 addr show wlan0` shows an `inet 192.168.x.y/24` (or whatever your home subnet is) |
| Container runs | `docker ps` shows the spooder container with `--net=host`                     |

Laptop-side: same Wi-Fi SSID, not a guest network, not a phone-tether.

### Step 1 — Pi: bring up the robot (with Foxglove already included)

The spooder bringup launch starts Foxglove bridge as one of its composable nodes:

```bash
docker exec -it spooder bash -lc "ros2 launch hexapod_nav full_pipeline.launch.py"
```

Inside the container, the Foxglove bridge announces itself on TCP/8765. From the laptop you can confirm it's listening:

```bash
nc -zv spooder-pi.local 8765
# expect: succeeded
```

### Step 2 — Laptop: open Foxglove Studio

1. Install Foxglove Studio from <https://foxglove.dev/download> (Linux / macOS / Windows all supported).
2. Launch the app.
3. Click **Open connection** → **WebSocket** → URL: `ws://spooder-pi.local:8765`.
4. The Topics panel populates with everything currently in the spooder ROS 2 graph: `/cmd_vel`, `/imu/data`, `/joint_states`, `/oak_d/color`, `/oak_d/points`, `/oak_d/stereo`, `/oak_d/imu`, plus whatever else the bringup publishes.
5. Plot panel: add `/cmd_vel` linear.x to a time-series panel. Move slider in Foxglove's `Teleop` (or publish directly with `ros2 topic pub`) — the trace responds in real time.

If your laptop refuses to resolve `spooder-pi.local`, fall back to the raw IP: `ws://192.168.x.y:8765` from `ip -4 addr show wlan0` on the Pi.

### Step 3 — Laptop: native DDS (optional, only if you want to publish commands from the laptop)

You can do both — keep Foxglove Studio for visualisation, and additionally install `ros-jazzy-ros-base` on the laptop for direct command-line publishing via native DDS. Steps:

```bash
sudo apt install ros-jazzy-ros-base
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

Verify (in a fresh terminal):

```bash
ros2 topic list
ros2 topic echo /cmd_vel --once
```

If you see topics printed on the laptop matching what the Pi publishes, native DDS discovery is working. You can now `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}' --rate 10` directly from the laptop, and Foxglove Studio will still see and plot that velocity over its WebSocket bridge — both clients view the same DDS domain.

### Which mode to pick?

| You want to…                                                               | Use                       |
| -------------------------------------------------------------------------- | ------------------------- |
| Just *watch* what the robot is doing (telemetry, camera, point clouds)     | Foxglove (Approach A)     |
| Publish one-off commands from a CLI without installing ROS 2 on the laptop | Either — both work        |
| Drive the robot from a Nav2 goal in `rviz2` running on the laptop          | Native DDS (Approach B)   |
| Share visualisation with a teammate who is not on the same Wi-Fi          | Foxglove + port-forward   |

---

## Gotcha: `--net=host` is mandatory for mDNS

Docker's default network is a bridge network — the container gets its own network namespace, with veth pair to a Linux bridge on the host. The mDNS responder inside the container cannot be reached by external hosts on the LAN, because its IP (`172.17.0.x` by default) is a Docker-internal address that the host masquerades rather than routes.

`docker run --net=host` shares the host's entire network namespace. The container binds to `wlan0` directly, picks up the host's hostname (the name returned by `hostname` is the host's hostname — `spooder-pi` if you've named the Pi that, or whatever you've set in `/etc/hostname`), inherits the mDNS responder the host already runs (Avahi on Raspbian), and is reachable at `spooder-pi.local` from the laptop over Wi-Fi with no further configuration.

This is exactly why the spooder container's launch instruction is `docker run --net=host ...`. If you ever see someone trying to run the container with the default bridge networking so the camera and IMU USB devices work "because the host-mode is too permissive" — they will discover that `spooder-pi.local` no longer resolves and that nothing on the laptop can see anything from the container. Don't switch to `--net=bridge`. The host networking is the price of mDNS; it's also the price of camera peripherals working without `--privileged`; this is one setting we want.

Verify the container is inheriting the host's hostname:

```bash
docker exec <container> hostname
# expect: spooder-pi (or whatever your Pi is called)
```

Verify mDNS resolves from another host on the LAN:

```bash
# from the laptop
ping -c1 spooder-pi.local
# expect: PING spooder-pi.local (192.168.x.y): 56 data bytes
#         64 bytes from 192.168.x.y: icmp_seq=0 ttl=64 time=...
```

If `.local` doesn't resolve, install `avahi-daemon` on the Pi (`sudo apt install avahi-daemon`) — that's the responder making the name available.

---

## Citations

- Foxglove bridge install + port: <https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2>
- ROS_DOMAIN_ID mechanism across machines: <https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/>
- `About-Domain-ID` (upstream ROS 2 docs): <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html>
