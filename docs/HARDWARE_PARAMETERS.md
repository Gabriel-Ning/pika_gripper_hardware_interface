# Hardware parameters (internal defaults)

User-facing knobs are only **`serial_port`** and **`max_width`** (see package
README). Everything below is a **built-in safety default** in
`PikaGripperInterface`. Do not expose these on bring-up launch files unless
you are debugging the plugin itself.

Override only by adding a matching `<param>` under the ros2_control
`<hardware>` block (or temporarily in a local xacro). Empty / omitted values
fall back to the defaults listed here.

## Units

ROS command/state and the filter use **finger travel** [m] (Piper-compatible).
Vendor wire quantity is **full opening width** ≈ `2 × finger_travel`.
Default `max_width` = `0.045` m travel ≈ `0.09` m opening.

## Transport

| Name | Default | Role |
|------|---------|------|
| `baudrate` | `460800` | Serial baud (device CH340). Other supported rates: 115200 / 230400 / 921600. |

## Lifecycle / enable

| Name | Default | Role |
|------|---------|------|
| `enable_on_activate` | `true` | Send ENABLE (11) in `on_activate`. |
| `disable_on_deactivate` | `true` | Send DISABLE (10) in `on_deactivate`. |
| `activation_timeout_s` | `2.0` | Max wait for first valid motor frame after ENABLE. |

## Command filter (write path, finger travel)

| Name | Default | Role |
|------|---------|------|
| `min_width` | `0.0` | Lower clamp [m] (finger travel). |
| `max_speed` | `0.15` | Per-cycle step limit source [m/s]: `Δ ≤ max_speed × period`. |
| `command_deadband` | `5e-4` | Skip re-sending targets within this delta [m]. |

`max_width` is user-facing (default `0.045`). It is also capped so
`2 × max_width ≤ protocol::max_width()` (~0.108 m opening).

## Feedback health (read path)

| Name | Default | Role |
|------|---------|------|
| `stale_warn_s` | `0.1` | Throttled WARN when last motor frame is older than this. |
| `stale_error_s` | `0.5` | `read()` returns `ERROR` (hardware fault path) past this age. |

## Safety behavior (not parameters)

These always apply and are not launch-tunable:

- Non-finite commands are rejected.
- First write after activate baselines to measured finger travel.
- Deactivate / destructor / `on_error` send DISABLE.
