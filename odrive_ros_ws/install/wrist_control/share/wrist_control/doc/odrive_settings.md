# ODrive Settings Guide

The ODrives are assumed to be **calibrated and ready**. This document covers
changing motion-control parameters and zeroing the encoder reference after a
hardware home.

---

## Connecting with odrivetool

```bash
pip install odrive          # if not already installed
odrivetool                  # connects via USB; auto-detects ODrives
```

If multiple ODrives are connected, select one:

```python
odrv = odrives[0]          # first detected
ax = odrv.axis0            # each ODrive has axis0 (and axis1 on dual-axis boards)
```

---

## Control mode

ODrive ControlMode enum:

| Value | Name | ROS param |
|-------|------|-----------|
| 1 | TORQUE_CONTROL | `control_mode: 1` |
| 2 | VELOCITY_CONTROL | `control_mode: 2` |
| 3 | POSITION_CONTROL | `control_mode: 3` ← default |

```python
from odrive.enums import ControlMode
ax.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv.save_configuration()
```

---

## Input mode

ODrive InputMode enum:

| Value | Name | Description |
|-------|------|-------------|
| 1 | PASSTHROUGH | Direct setpoint, no trajectory smoothing |
| 2 | VEL_RAMP | Velocity ramp (velocity mode only) |
| 5 | TRAP_TRAJ | Trapezoidal trajectory — uses vel/accel limits |

```python
from odrive.enums import InputMode
ax.controller.config.input_mode = InputMode.TRAP_TRAJ
odrv.save_configuration()
```

Then set the matching ROS parameter:

```bash
ros2 run wrist_control hsa_odrive_control --ros-args -p input_mode:=5
```

---

## Trapezoidal trajectory limits

These determine how quickly the ODrive accelerates and decelerates when
`input_mode = TRAP_TRAJ`. Units are **turns/s** and **turns/s²**.

```python
ax.trap_traj.config.vel_limit   = 2.0   # max velocity (turns/s)
ax.trap_traj.config.accel_limit = 4.0   # max acceleration
ax.trap_traj.config.decel_limit = 4.0   # max deceleration
odrv.save_configuration()
```

Repeat for each axis/ODrive. For the wrist (6 axes on 3 dual-axis ODrives):

```python
for ax in [odrv0.axis0, odrv0.axis1, odrv1.axis0, odrv1.axis1, odrv2.axis0, odrv2.axis1]:
    ax.controller.config.input_mode = InputMode.TRAP_TRAJ
    ax.trap_traj.config.vel_limit   = 2.0
    ax.trap_traj.config.accel_limit = 4.0
    ax.trap_traj.config.decel_limit = 4.0
odrv0.save_configuration()
odrv1.save_configuration()
odrv2.save_configuration()
```

---

## Current limit

```python
ax.motor.config.current_lim = 10.0      # Amperes
ax.motor.config.current_lim_margin = 8.0
odrv.save_configuration()
```

---

## Velocity limit (position and velocity modes)

A separate velocity limit applies in position and velocity control modes:

```python
ax.controller.config.vel_limit = 5.0    # turns/s
odrv.save_configuration()
```

---

## Zeroing encoders via ROS service

After positioning the wrist at its physical home:

```bash
ros2 service call /hsa_odrive_control/zero_encoders std_srvs/srv/Empty '{}'
```

The node reads each ODrive's current position estimate and stores a software
offset so that the current physical position maps to `zero_point` defined in
`wrist.yaml` (default `0.0°` for all axes). No data is written to the ODrive;
the offset lives in the running node only — re-zero after each restart.

### Persistent hardware zero (optional)

If you want the ODrive encoder to read zero at the home position permanently,
use `odrivetool`:

```python
# With axis in CLOSED_LOOP_CONTROL, drive to home position, then:
ax.set_abs_pos(0.0)         # sets current position as absolute zero
odrv.save_configuration()
```

After this, the ROS `~/zero_encoders` service will record offset = 0 (a no-op)
at the home position, which is the cleanest setup.

---

## Checking axis state from ROS

```bash
ros2 topic echo /odrive_axis0/controller_status
```

Key fields:
- `axis_state`: 8 = CLOSED_LOOP_CONTROL, 1 = IDLE
- `active_errors`: 0 means no errors
- `pos_estimate`: current position in turns

---

## Saving and rebooting

Always call `save_configuration()` after changing settings, then reboot:

```python
odrv.save_configuration()   # writes to non-volatile memory
odrv.reboot()               # apply changes
```

Wait ~5 s for the ODrive to reboot before reconnecting with `odrivetool`.
