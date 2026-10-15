# wrist_control — Usage

## Overview

`hsa_odrive_control` is a ROS2 node that bridges the HSA wrist kinematics
model (`hsa_robots`) and the per-axis ODrive CAN driver nodes
(`odrive_can_node`). It accepts joint extension angles in **degrees** and
publishes `ControlMessage` packets to each ODrive axis.

```
your publisher ──► ~/joint_commands ──► hsa_odrive_control ──► /odrive_axisN/control_message ──► odrive_can_node ──► CAN
```

---

## Prerequisites

1. Build and source the workspace:

   ```bash
   cd ~/wrist_26/odrive_ros_ws
   colcon build --packages-select odrive_can wrist_control
   source install/setup.bash
   ```

2. The `hsa_robots` library must be installed in your Python environment:

   ```bash
   uv pip install -e ~/wrist_26/hsa_robots/
   ```

---

## Launching

### Full system (ODrive nodes + wrist_control)

```bash
ros2 launch wrist_control wrist_control.launch.py
```

This starts six `odrive_can_node` instances (axes 0–5 on `can0`) **and** the
`hsa_odrive_control` node.

### Wrist control node only (ODrives already running)

```bash
ros2 run wrist_control hsa_odrive_control \
  --ros-args -p robot_config:=/path/to/wrist.yaml
```

---

## Start-up procedure

1. **Launch the system** (see above).
2. **Enable the axes** — puts all ODrives in `CLOSED_LOOP_CONTROL`:

   ```bash
   ros2 service call /hsa_odrive_control/enable std_srvs/srv/Trigger '{}'
   ```

3. **Move the wrist to its physical home position** (all tendons at zero
   extension).

4. **Zero the encoders** — records the current ODrive position estimates as the
   software zero for each axis:

   ```bash
   ros2 service call /hsa_odrive_control/zero_encoders std_srvs/srv/Empty '{}'
   ```

5. **Send joint commands** (see below).

---

## Sending joint commands

Publish a `Float32MultiArray` with 6 extension angles in **degrees**, one per
HSA axis in order (index 0–5):

```bash
ros2 topic pub --once /hsa_odrive_control/joint_commands \
  std_msgs/msg/Float32MultiArray \
  '{data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]}'
```

- Values are clamped to the range defined in `wrist.yaml` and
  **hard-clamped** to `[-180°, +540°]` regardless of config.
- Handedness is handled automatically: CW axes receive the angle directly;
  CCW axes receive `max_angle − angle`.

---

## Disabling

```bash
ros2 service call /hsa_odrive_control/disable std_srvs/srv/Trigger '{}'
```

---

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `robot_config` | string | `hsa_robots/wrist.yaml` | Path to robot YAML config |
| `control_mode` | int | `3` | ODrive ControlMode (3=position, 2=velocity, 1=torque) |
| `input_mode` | int | `1` | ODrive InputMode (1=passthrough, 5=trap_traj) |
| `vel_ff` | float | `0.0` | Velocity feed-forward (turns/s) in position mode |
| `torque_ff` | float | `0.0` | Torque feed-forward (Nm) |

Override at launch time:

```bash
ros2 run wrist_control hsa_odrive_control \
  --ros-args \
  -p control_mode:=3 \
  -p input_mode:=5 \
  -p vel_ff:=0.0
```

### Enabling trapezoidal trajectory (smooth motion)

Set `input_mode:=5` (TRAP_TRAJ) and pre-configure the velocity/accel limits
on each ODrive — see [odrive_settings.md](odrive_settings.md).

---

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/zero_encoders` | `std_srvs/Empty` | Snapshot current positions as the physical (software) zero; saves `zero_offset` to `hsa_robots/wrist.yaml` and publishes `/hsa_zero_offsets` |
| `~/enable` | `std_srvs/Trigger` | CLOSED_LOOP_CONTROL on all axes |
| `~/disable` | `std_srvs/Trigger` | IDLE on all axes |

---

## Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `~/joint_commands` | Sub | `std_msgs/Float32MultiArray` | 6 extension angles (degrees) |
| `~/jog` | Sub | `std_msgs/Float32MultiArray` | 6 per-axis steps (degrees, 0 = no change); may go below 0, down to −180° |
| `/hsa_zero_offsets` | Pub (latched) | `std_msgs/Float32MultiArray` | Software zero offset per axis, motor turns |
| `/odrive_axisN/control_message` | Pub | `odrive_can/ControlMessage` | Per-axis ODrive command |
| `/odrive_axisN/controller_status` | Sub | `odrive_can/ControllerStatus` | Per-axis position feedback |

---

## Testing

```bash
colcon test --packages-select wrist_control
colcon test-result --verbose
```

---

## Robot config (wrist.yaml)

The config lives at `~/wrist_26/hsa_robots/wrist.yaml` and nodes read it from
the `hsa_robots` package; `config/wrist.yaml` here is a symlink to it. Each
entry maps to one ODrive axis:

```yaml
hsas:
  - handedness: CW    # clockwise — motor angle = extension angle
    min_angle: 0.0
    max_angle: 360.0
    zero_point: 0.0
    index: 0
    motor: odrive
  ...
```

CAN bus node IDs match `index` (axis 0 → node_id 0, etc.).
