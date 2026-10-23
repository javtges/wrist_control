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
sudo ip link set can0 txqueuelen 1000      # default 10 overflows when six setpoints go out at once
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
- **All joints (°) + Set All** — every slider to the typed angle (clamped per
  joint) and one command immediately. **⤒ Extend Fully** does the same with
  each joint's `max_angle` (360° in `wrist.yaml`): the fully extended wrist
  for data collection. The ODrives hold a position setpoint, so a single
  command keeps the pose; publishing need not be on.
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

## Step 4b — One stamped state topic: `wrist_state_publisher`

`wrist_control.launch.py` also starts `wrist_state_publisher` (`state:=false` to
skip, `state_rate:=` Hz, default 100). It folds `/hsa_odrive_control/joint_states`
and `joint_commands` plus every axis's `controller_status` and `odrive_status`
into one `hsa_wrist_msgs/WristState` on `/hsa/wrist/state`: extension and
command angles (deg), ODrive position (turns), velocity, torque target and
estimate, Iq setpoint and measured, axis state, errors, bus voltage and current,
FET and motor temperatures, disarm reason, each per axis, plus how old each
source's newest sample was (`*_age_s`; the ODrive messages carry no stamp, so
everything is keyed on receive time and the header is the publish time). This
is what HSA-UMI's recorder stores per camera frame and what rollout bags carry.

```bash
ros2 run wrist_control wrist_state_publisher          # on its own
ros2 topic hz /hsa/wrist/state                        # ~100 Hz
ros2 topic echo --once /hsa/wrist/state --field extension_deg
```

The `hsa_wrist_msgs` package is in this workspace; build it before
`wrist_control` (`colcon build --packages-select hsa_wrist_msgs wrist_control`).

## Step 5 — Camera tracking of the platform

A baseplate USB camera tracks the four AprilTags on the platform (16h5, ids
0–3). `platform_pose` publishes the platform's pose relative to its zero on
`/platform_pose`, `plate_guard` checks that the top plate is mounted the
expected way round, and `tag_monitor` shows the camera view with the motor
positions, saving screenshots + positions whenever the tags are lost. See
[camera_tracking.md](../../camera_tracking/doc/camera_tracking.md).

`ros2 launch wrist_control wrist_control.launch.py` starts all of this with
the wrist (`camera:=false` to skip it, `show_window:=false` for no OpenCV
window). On its own:

```bash
ros2 launch camera_tracking tracking.launch.py   # Arducam OV9281 by its /dev/v4l/by-id path, 1280x800 @ 120 fps
```

**Plate mounting check.** The terminal prints `plate OK` or an ERROR such as
`plate WRONG: plate turned +120 deg (one slot clockwise); ID1 9 o'clock,
ID2 1 o'clock` (repeated as a WARN every 5 s), and `joint_commander_gui`
shows the same text in a banner at the top: green OK, red wrong, grey when the
camera stack is not running. Expected mounting: ID1 at 9 o'clock and ID2 at
1 o'clock in the image (ID3 at the bottom). If the camera is ever remounted,
run `plate_check`, read ID1's bearing and put it in
`camera_tracking/config/platform.yaml` as `expected_id1_bearing_deg`.

After mounting the top plate, `ros2 run camera_tracking plate_check` reports
the clock position of each tag in the image (ID1 should be at 12 o'clock with
the camera upright) and, with `-p drive:=true`, which leg pair sits under each
tag.

Zero the camera once per mount: with the motors enabled, encoders zeroed and
every joint commanded to 0° (plate flat on the table), press **⌖ Set Platform
Zero (camera)** in `joint_commander_gui` (it only enables while all HSAs read
0 ± 1°), or `ros2 service call /set_platform_zero std_srvs/srv/Trigger`. The
zero is saved to `~/.config/camera_tracking/platform_zero.yaml` and reloaded
on later starts.

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
