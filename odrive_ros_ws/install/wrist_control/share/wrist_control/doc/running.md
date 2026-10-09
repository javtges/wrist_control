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

Motors 0, 2, 4:

`//////    \\\\\\`
`//////    \\\\\\`
`//////    \\\\\\`
`//////    \\\\\\`


Motors 1, 3, 5:

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
ros2 run wrist_control hsa_odrive_control \
    --ros-args -p robot_config:=$HOME/wrist_26/odrive_ros_ws/src/wrist_control/config/wrist.yaml
```

Key parameters (can also be set at launch or at runtime via `ros2 param set`):

| Parameter      | Default | Description                              |
|----------------|---------|------------------------------------------|
| `robot_config` | `""`    | Absolute path to `wrist.yaml`            |
| `control_mode` | `3`     | 3=position, 2=velocity, 1=torque         |
| `input_mode`   | `1`     | 1=passthrough, 5=trap_traj               |
| `vel_ff`       | `0.0`   | Velocity feedforward (turns/s)           |
| `torque_ff`    | `0.0`   | Torque feedforward (Nm)                  |

Services published by this node:

| Service                            | Type     | Effect                              |
|------------------------------------|----------|-------------------------------------|
| `/hsa_odrive_control/zero_encoders`| `Empty`  | Snapshot current positions as zero  |
| `/hsa_odrive_control/enable`       | `Trigger`| Enable motor closed-loop control    |
| `/hsa_odrive_control/disable`      | `Trigger` | Disable motors (idle state)        |

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
- The **vertical slider** sets the commanded extension angle in degrees.
- The **bar** shows the current slider value visually. Blue = CW, pink = CCW.

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
- **Set Zero Here** — calls `/hsa_odrive_control/zero_encoders`; the node
  records current motor positions as the new zero reference (motors do not move).
- **Enable / Disable** — calls the `enable`/`disable` services on the node.

---

## Step 4 (alternative) — SpaceMouse Commander

Drives the joints from a 3Dconnexion SpaceMouse instead of the sliders. It uses
the Stewart-platform kinematics from `resource/rigid_dynamics.py` and publishes
to `/hsa_odrive_control/joint_commands` the same way as the GUI: 6 angles in
degrees, clamped to each joint's `min_angle`–`max_angle` from `wrist.yaml`,
at 20 Hz, and only while publishing is on. Publishing starts **off**.

Needs `easyhid` in the active Python environment
(`uv pip install easyhid==0.0.10`) and read/write access to the SpaceMouse's
`/dev/hidraw*` (the `plugdev` udev rule).

```bash
ros2 run wrist_control spacemouse_commander
```

| Control | Effect |
|---------|--------|
| Left button | Toggle publishing on/off |
| Right button | Go to Zero (publish each `zero_point` once), then publishing off |
| `ros2 service call /spacemouse_commander/set_publishing std_srvs/srv/SetBool '{data: true}'` | Publishing on/off from the terminal |

Preview the output without moving anything (published even while publishing is off):

```bash
ros2 topic echo /spacemouse_commander/joint_preview
```

With the puck at rest every joint reads **35.14°**, not 0°. That is what the
rigid_dynamics model gives for the home pose, so turning publishing on moves all
joints there. Only z translation, roll, pitch and yaw are used (as in
rigid_dynamics.py); x/y translation are ignored.

Parameters: `publish_rate` (20), `start_publishing` (false), `max_rot` (π/5 rad),
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
6. Click [Set Zero Here] with motors at their physical home position
7. Toggle [PUBLISHING] on — use sliders to command joints
8. When done: click [Go to Zero] → toggle [PUBLISHING] off → click [Disable]
```

---

## Troubleshooting

**GUI starts but parameter changes have no effect**
The `hsa_odrive_control` node was not running when the GUI launched.
Restart the GUI after the node is up.

**"zero_encoders service not ready"** in node log
The `hsa_odrive_control` node is not running or is not yet initialised.

**Motors jerk on first enable**
The zero has not been set. Click **Set Zero Here** before enabling, or
position the motors at zero physically before enabling.

**No `/hsa_odrive_control/joint_commands` topic visible**
Publishing is toggled off. Click the toggle button in the GUI.
