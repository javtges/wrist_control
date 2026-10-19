# Running the HSA Wrist System

## Prerequisites

- Ubuntu 24.04, ROS2 Jazzy
- ODrive Pro/S1 with CAN interface (`can0`)
- `wrist-odrive` virtualenv active (`source ~/wrist_26/wrist-odrive/bin/activate`)
- Workspace built and sourced

```bash
cd ~/wrist_26/odrive_ros_ws
colcon build --packages-select wrist_control odrive_can
source install/setup.bash
```

---

## Step 0 -- Adding the HSAs:

When viewing each set of two struts on the wrist:

Holes of the HSAs should be facing out perpendicularly from the line connecting the two big gears, as close to parallel as possible with each other.

Motors 1, 3, 5:

`//////    \\\\\\`

`//////    \\\\\\`

`//////    \\\\\\`

`//////    \\\\\\`


Motors 0, 2, 4:

`\\\\\\    //////`

`\\\\\\    //////`

`\\\\\\    //////`

`\\\\\\    //////`


## Step 1 — Bring up the CAN bus

```bash
sudo ip link set can0 up type can bitrate 500000
```

To bring it down when finished:

```bash
sudo ip link set can0 down
```

---

## Step 2 — Launch the ODrive CAN bridge

This launches one `odrive_can_node` per HSA axis (0–5), each translating ROS2
messages into CAN frames for its ODrive controller on `can0`.

```bash
ros2 launch odrive_can wrist_launch.yaml
```

Verify it is running:

```bash
ros2 node list        # should show /odrive_axis0/can_node ... /odrive_axis5/can_node
```

---

## Step 3 — Launch the HSA ODrive control node

```bash
ros2 run wrist_control hsa_odrive_control
```

The robot config is always `~/wrist_26/hsa_robots/wrist.yaml`. Every node reads
it from the `hsa_robots` package by default, and `config/wrist.yaml` in this
package is a symlink to it. Edit that one file; no rebuild needed.

Key parameters (can also be set at launch or at runtime via `ros2 param set`):

| Parameter      | Default | Description                              |
|----------------|---------|------------------------------------------|
| `robot_config` | `""`    | Override path; default `hsa_robots/wrist.yaml` |
| `control_mode` | `3`     | 3=position, 2=velocity, 1=torque         |
| `input_mode`   | `1`     | 1=passthrough, 5=trap_traj               |
| `vel_ff`       | `0.0`   | Velocity feedforward (turns/s)           |
| `torque_ff`    | `0.0`   | Torque feedforward (Nm)                  |

Services published by this node:

| Service                            | Type     | Effect                              |
|------------------------------------|----------|-------------------------------------|
| `/hsa_odrive_control/zero_encoders`| `Empty`  | Set the physical zero (GUI's **Set Physical Zero**) |
| `/hsa_odrive_control/enable`       | `Trigger`| Enable motor closed-loop control    |
| `/hsa_odrive_control/disable`      | `Trigger` | Disable motors (idle state)        |

### Position and zero topics

| Topic | Published by | Contents |
|-------|--------------|----------|
| `/odrive_positions` | `odrive_position_monitor` | Raw ODrive `pos_estimate` per axis, motor turns |
| `/hsa_zero_offsets` | `hsa_odrive_control` (latched) | Software zero offset per axis, motor turns |
| `/hsa_positions` | `odrive_position_monitor` | `odrive_positions + hsa_zero_offsets`, converted by hsa_robots (2:1 gear, handedness) to extension degrees. Reads 0 at the physical zero and 0–360 across the slider range |

The software zero is stored per HSA as `zero_offset` (motor turns) in
`hsa_robots/wrist.yaml`. `hsa_odrive_control` loads it at startup, and
**Set Physical Zero** / **Clear Physical Zero…** write it back, so the zero
survives restarting the ROS nodes.

The offset is only correct while each ODrive's `pos_estimate` is unchanged
from when the zero was set. If an ODrive reboots or loses power, its encoder
position can come back different (the motor turns more than once across the
0–360° range), and the saved offset will then be off. After any ODrive power
cycle, check `/hsa_positions` against the hardware, and if it's off, jog to the
physical zero and press **Set Physical Zero** again.

Topic consumed:

| Topic                                   | Type                 | Layout                                  |
|-----------------------------------------|----------------------|-----------------------------------------|
| `/hsa_odrive_control/joint_commands`    | `Float32MultiArray`  | One `float32` per joint, extension (°) |

---

## Step 4 — Launch the Joint Commander GUI

```bash
ros2 run wrist_control joint_commander_gui
```

To use a custom config:

```bash
ros2 run wrist_control joint_commander_gui \
    --ros-args -p robot_config:=/path/to/wrist.yaml
```

### GUI overview

**Joint panels** (top row) — one column per HSA joint.
- The **vertical slider** sets the commanded extension angle in degrees,
  0–360 measured from the physical zero. Sliders cannot go below 0.
- The **−2° / +2° buttons** under each slider step that motor's setpoint by 2°
  from where it is held. They are the only control that can drive a motor
  below 0, past the current zero (down to −180°), so you can reach a physical
  zero that is behind where the motor is now. A jog button moves only its own
  HSA; **Link** affects the sliders only. Pressing a jog button turns
  PUBLISHING off, so the sliders don't immediately pull the motor back.
- The **bar** shows the current slider value visually. Blue = CW, pink = CCW.
- **meas** is the live position from `/hsa_positions`.

**ODrive Parameters** (bottom-left):
- **Control mode** dropdown — changes `control_mode` on the running node.
- **Input mode** dropdown — changes `input_mode` on the running node.
- **Vel FF** slider — velocity feedforward; click the `0` button to zero it.
- **Torque FF** slider — torque feedforward; click the `0` button to zero it.

> Parameter changes are sent to the `hsa_odrive_control` node via `AsyncParametersClient`.
> The node must be running for changes to take effect.

**Actions** (bottom-right):
- **PUBLISHING / NOT PUBLISHING** toggle — starts/stops the publish loop.
  Set a rate (Hz) in the spinbox before enabling.
- **Go to Zero** — moves all sliders to `zero_point` from the config and
  sends one command immediately (regardless of publish state).
- **Set Physical Zero** — calls `/hsa_odrive_control/zero_encoders`; the node
  records current motor positions as the physical zero (motors do not move),
  saves it to `zero_offset` in `hsa_robots/wrist.yaml`, and resets the sliders
  to 0 to match. This is the only place the zero is set.
- **Clear Physical Zero…** — after a confirmation, deletes the saved zero for
  every HSA (all `zero_offset` back to 0 in `wrist.yaml`), so positions are
  measured from each ODrive's raw encoder zero. The motors do not move. The
  old values are printed in the `hsa_odrive_control` log, so they can be typed
  back into `wrist.yaml` if it was a mistake (only valid if the ODrives
  haven't rebooted since).
- **Enable / Disable** — calls the `enable`/`disable` services on the node.

---

## Step 4 (alternative) — SpaceMouse Commander

Drives the joints from a 3Dconnexion SpaceMouse instead of the sliders. It uses
the Stewart-platform kinematics from `resource/rigid_dynamics.py` and publishes
to `/hsa_odrive_control/joint_commands` the same way as the GUI: 6 angles in
degrees, clamped to the `HSARobot` action space (0–360 from the physical zero),
at 100 Hz, and only while publishing is on. Publishing starts **off**.

Needs `easyhid` in the active Python environment
(`uv pip install easyhid==0.0.10`) and read/write access to the SpaceMouse's
`/dev/hidraw*` (the `plugdev` udev rule).

```bash
ros2 run wrist_control spacemouse_commander
```

| Control | Effect |
|---------|--------|
| Left button | Toggle publishing on/off |
| Right button | Enable / disable the motors: disables if any axis is in closed loop, otherwise enables. Turns publishing off first either way |
| `ros2 service call /spacemouse_commander/set_publishing std_srvs/srv/SetBool '{data: true}'` | Publishing on/off from the terminal |

Preview the output without moving anything (published even while publishing is off):

```bash
ros2 topic echo /spacemouse_commander/joint_preview
```

With the puck at rest every joint reads **35.14°**, not 0°. That is what the
rigid_dynamics model gives for the home pose, so turning publishing on moves all
joints there. Only z translation, roll, pitch and yaw are used (as in
rigid_dynamics.py); x/y translation are ignored.

Parameters: `publish_rate` (100), `start_publishing` (false), `max_rot` (π/5 rad),
`max_lin` (10 cm), `clockwise` (true: ODrives 0–5 clockwise from above, motor 0
at rigid_dynamics.py's leg 0, so (0,1), (2,3), (4,5) are apart on the base and
meet at the top), `leg_offset` (0: rotate the motor numbering by this many legs).

---

## Typical workflow

```
1. Bring up CAN (Step 1)
2. Launch odrive_can bridge (Step 2)
3. Launch hsa_odrive_control (Step 3)
4. Launch joint_commander_gui (Step 4)
5. Click [Enable] in the GUI
6. Use each HSA's [−2°]/[+2°] buttons to bring it to where its physical zero
   should be (watch the hardware; the jog may go below the current zero)
7. Click [Set Physical Zero] — every HSA now reads 0 and the sliders reset to 0
8. Toggle [PUBLISHING] on — the sliders now command 0–360° from that zero
9. When done: click [Go to Zero] → toggle [PUBLISHING] off → click [Disable]
```

---

## Troubleshooting

**GUI starts but parameter changes have no effect**
The `hsa_odrive_control` node was not running when the GUI launched.
Restart the GUI after the node is up.

**"zero_encoders service not ready"** in node log
The `hsa_odrive_control` node is not running or is not yet initialised.

**Motors jerk on first enable**
The zero has not been set. Enable, jog to the physical zero, and click
**Set Physical Zero** before turning PUBLISHING on.

**No `/hsa_odrive_control/joint_commands` topic visible**
Publishing is toggled off. Click the toggle button in the GUI.
