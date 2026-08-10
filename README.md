# pika_gripper_hardware_interface

ros2_control `SystemInterface` plugin for the AgileX Pika gripper over its
USB serial link. Follows the same end-effector conventions as Piper
(finger-travel ROS units, background serial I/O, write-path filters).

## Overview

```text
ros2_control lifecycle
  → PikaGripperInterface (reader/writer background threads)
    → pika_protocol (frame parse / command encode / width kinematics)
      → serial 460800 8N1 (CH340) → Pika MCU
```

Protocol reverse-engineered from `agilexrobotics/pika_sdk`
(`pika/serial_comm.py`, `pika/gripper.py`) and verified against a live
device. Commands are binary (`[type:u8][value:f32 LE]\r\n`); feedback is a
pseudo-JSON stream (trailing commas; EMPTY field values while the motor is
disabled). The commanded quantity on the wire is the motor angle [rad].

**ROS joint units match Piper:** single-finger prismatic travel [m]. The
plugin converts with `opening_width = 2 × finger_travel` before the linkage
inverse (and the reverse on read). Default travel range is **0–0.045 m**
(≈ 0–0.09 m full opening).

The integrated fisheye camera and RealSense D405 are NOT part of this
plugin — they are plain UVC / RealSense devices on the same USB hub, driven
by `usb_cam` / `realsense2_camera` from a host bring-up package.

## ros2_control Contract

|          | Count | Interfaces                                    | Units |
|----------|-------|-----------------------------------------------|-------|
| Command  | 1     | `position`                                    | m (finger travel, 0–0.045) |
| State    | 3     | `position`, `velocity`, `effort`              | m, m/s, **mA (motor phase current — grip-force proxy)** |
| State (unlisted diagnostics) | 4 | `voltage`, `driver_temp`, `motor_temp`, `fault` | V, °C, °C, status bits |

`effort` deliberately reports the raw motor phase current (mA) instead of
fingertip force in N: the vendor publishes no torque constant, and the
current→force map varies with the linkage angle. Consumers that need newtons
should calibrate a lookup table (force gauge at several openings). For
learning pipelines the raw signal is sufficient and consistent.

Joint name is URDF-defined (canonical: `gripper_left_joint`, or
`${prefix}gripper_left_joint`). Compatible with
`parallel_gripper_action_controller/GripperActionController`.

## User parameters

Only these two are meant for bring-up / host composition:

| Name | Required | Default | Description |
|------|----------|---------|-------------|
| `serial_port` | no | `/dev/ttyUSB0` | Prefer `/dev/serial/by-id/usb-1a86_USB_Serial-*` |
| `max_width` | no | `0.045` | ROS finger-travel upper clamp [m]; wire opening ≈ 2× |

Example (after composing this plugin in a bring-up / URDF with
`serial_port` / `max_width`):

```bash
# From a host launch that passes the two user params, e.g.:
#   serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
#   max_width:=0.045
```

All other knobs (baudrate, enable/disable on lifecycle, rate limits, deadband,
stale thresholds, …) are **fixed safety defaults** inside the plugin. See
[`docs/HARDWARE_PARAMETERS.md`](docs/HARDWARE_PARAMETERS.md).

## Safety Guards

| Guard | Behavior |
|-------|----------|
| Non-finite command | rejected (no send) |
| Travel limits | clamped to `[0, max_width]` (finger travel) |
| First write | baseline = measured travel (synced in `on_activate`) |
| Per-cycle step | rate-limited by the built-in filter |
| Deadband | unchanged targets are not re-sent |
| Stale feedback | warn, then hardware `ERROR` past built-in thresholds |
| Deactivate / destructor / on_error | DISABLE sent; destructor is the safety net |

## Realtime

`read()` copies `std::atomic` caches; `write()` runs the pure command filter
and posts to a condition variable. All serial I/O lives in the reader/writer
background threads. The plugin adds no meaningful load to the shared
controller_manager loop.

## Build / Test (offline)

```bash
colcon build --packages-select pika_gripper_hardware_interface
colcon test --packages-select pika_gripper_hardware_interface
# protocol, kinematics, and write-guard suites need no hardware
```

## Design Notes / Known Issues

- **Motor needs separate 24V on XT30(PB).** USB Type-C only powers the MCU and
  cameras. Without 24V the device streams empty `"motor":,` frames forever
  and activation fails with `No valid motor feedback within ... s after ENABLE`.
- **Feedback is empty until the motor reports.** A freshly powered gripper
  streams `{"motor":,"motorstatus":}`. `on_activate` sends ENABLE and then
  requires a valid frame within the activation timeout; activation fails
  (and DISABLEs) otherwise.
- **Measure travel at bring-up** if FinRay tips differ; set `max_width` and
  `config/joint_limits.yaml` together.
- **`fault` state interface** exposes the raw `Status` hex bits as a double;
  bit semantics are undocumented by AgileX. Treat nonzero as "inspect".
- Feedback `velocity` is derived from motor `Speed` through the linkage
  Jacobian (numeric), scaled to finger travel, not measured at the fingers.
