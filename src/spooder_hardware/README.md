# spooder_hardware

Hardware interface for the Spooder hexapod's ST3215 servo motors.

## Contents

- `src/` — C++ hardware interface implementation
- `include/` — Header files
- `config/` — Hardware configuration
- `launch/` — Hardware bringup launch files

## Hardware Interface

- **Servo model:** ST3215 (Feetech, serial bus)
- **Bus:** Half-duplex serial (one wire for all 18 servos)
- **Communication:** 1 Mbps, 1-byte address
- **Position feedback:** 0–4095 ticks (0–360°)

## Files

| File | Purpose |
|------|---------|
| `st3215_communication.hpp/cpp` | Serial bus communication |
| `st3215_hardware_interface.hpp/cpp` | ros2_control hardware interface |
| `config/controllers.yaml` | ros2_control config |
| `config/hardware.yaml` | Hardware parameters |

## Launch

```bash
ros2 launch spooder_hardware bringup.launch.py
```

## Wiring

ST3215 servos are daisy-chained on a single half-duplex serial bus. Connect the bus to a USB-to-serial adapter (e.g., FTDI), then to the host.

## Dependencies

- `ros2_control`
- `controller_manager`
- `hardware_interface`
- `spooder_description`
