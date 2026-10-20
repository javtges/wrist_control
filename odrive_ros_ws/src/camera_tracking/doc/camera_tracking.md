# camera_tracking — baseplate camera + AprilTag platform tracking

A USB camera on the baseplate looks up at four AprilTags on the moving
platform. This package drives the camera (`usb_cam`), undistorts the image
(`image_proc`), detects the tags (`apriltag_ros`), publishes the platform pose
relative to a stored zero (`platform_pose`) and runs a monitor that shows the
camera view with the motor positions drawn on it and saves screenshots + motor
positions whenever the tags are lost (`tag_monitor`).

```
usb_cam ──► /tracking_cam/image_raw ──► rectify ──► /tracking_cam/image_rect ──► AprilTagNode
            /tracking_cam/camera_info ─────────────────────┐                      │
                                       TF camera→tag16h5:<id>  ◄──────────────────┤
                                       /apriltag/detections    ◄──────────────────┘
                                                   │
   platform_pose ◄── detections + camera_info      └─► tag_monitor ◄── detections + image_rect
     + /hsa_odrive_control/joint_states (zero guard)      + /hsa_odrive_control/joint_states
     /platform_pose          (in platform_zero)           + /odrive_positions
     /platform_pose_camera   (in camera)                  + /hsa_odrive_control/joint_commands
     /platform_tag_visible, /platform_source_tag          ──► ~/wrist_captures/*.png,*.yaml,captures.csv
     TF camera→platform_zero (static) →platform
     service /set_platform_zero
```

## The plate's tags and the platform frame

Family **16h5**, ids **0–3**, 12 mm black square (18 mm with the white border):

| id | where | offset from ID0 (mm, ID0 frame) |
|----|-------|-----------------------------|
| 0 | middle | (0, 0) |
| 1 | top | (0, +40) |
| 2 | lower right | (+34.64, −20) |
| 3 | lower left | (−34.64, −20) |

All three outer tags are 40 mm centre to centre from ID0, 120° apart, and are
printed with the same orientation as ID0. The **platform frame is the ID0 tag
frame**: origin at the centre of ID0, x to the right and y up along the printed
tag, z out of the plate towards the camera.

`platform_pose` does its own pose estimate from `/apriltag/detections` (the
corner pixels) and `/tracking_cam/camera_info`, so it can apply the hierarchy:

1. **ID0 visible** → pose from ID0's four corners.
2. **ID0 not visible** → pose from the first visible outer tag in `tag_priority`
   (default 1, 2, 3), shifted by that tag's offset.

The layout lives in `config/platform.yaml` (`tag_ids`, `tag_x_mm`, `tag_y_mm`,
`tag_size`). If the plate is printed differently, change the numbers there,
not the code. `pose_source: all` instead refines one pose over every visible
tag's corners (anchored on the priority tag), which is less noisy; the default
`priority` follows the hierarchy literally.

**Checking the layout on hardware:** with more than one tag visible the node
publishes `/platform_pose/tag_consistency_mm`, one value per tag: how far the
platform origin computed from that tag alone lands from the published pose.
Values of a few tenths of a millimetre are corner noise; a steady offset on
one tag means its `tag_x_mm`/`tag_y_mm` (or its printed orientation) is wrong.

## Is the plate mounted the right way round?

`plate_check` answers two things while the tracking stack runs:

```bash
ros2 run camera_tracking plate_check                        # report only; you move the pairs
ros2 run camera_tracking plate_check --ros-args -p drive:=true   # moves each leg pair itself
```

1. **Where the tags are in the image**: the clock position of ID1, ID2 and
   ID3 around the plate centre (12 o'clock = top of the image), and whether
   the outer tags run ID1 → ID2 → ID3 clockwise 120° apart as in the
   drawing. With the camera mounted upright on the baseplate, "tag 1 on top"
   is ID1 at 12 o'clock. The clockwise order can only come out wrong if the
   tags are printed differently from the drawing; a plate rotated by 120°
   keeps the order and moves ID1 to 4 or 8 o'clock.
2. **Which leg pair is under each tag**: moving one pair of HSAs tilts the
   plate, and the tag whose distance to the camera changes most is the one
   above that pair. With `drive:=true` the node commands pairs (0,1), (2,3),
   (4,5) to `wiggle_deg` (20°) in turn, `settle_s` (2 s) each, returns to 0
   and prints `leg pair -> tag above it`. Without it, take the plate to rest
   first (the rest distances are snapshotted when all four tags are first
   visible), move one pair from `joint_commander_gui` and read "ID<k> moved
   most" in the report. Compare with the pair the design puts under ID1.

The check needs all four tags visible at rest and an approximate
`camera_info.yaml` (distances are only compared to each other).

## Guarding the mounting at every launch

`plate_guard` (part of `tracking.launch.py`, so also of
`wrist_control.launch.py`) compares the live tag bearings with the expected
mounting and reports continuously:

- terminal: `plate OK ...` once on start, `plate WRONG: plate turned +120 deg
  (one slot clockwise); ID1 9 o'clock, ID2 1 o'clock` as an ERROR on the
  change and a WARN every 5 s while it stays wrong, `plate NO TAGS ...` when
  nothing is detected, `plate MIRRORED ...` if the tags run the wrong way;
- `/plate_mount_ok` (Bool) and `/plate_mount_status` (String) at 2 Hz;
- `joint_commander_gui` shows the status as a banner across the top: green
  OK, red otherwise, grey when the camera stack is not running.

The expected mounting is one number in `config/platform.yaml`:
`expected_id1_bearing_deg`, ID1's bearing around the plate centre, clockwise
from the top of the image (283.5 = ID1 at 9 o'clock, ID2 at 1, ID3 at the
bottom, as measured on the approved mounting on 2026-10-20). One visible outer tag is enough, so the tag clipped
at the frame edge does not matter. If the camera is remounted, read the new
bearings from `plate_check` and update the number. `tolerance_deg` (15) is
the allowed error before WRONG.

## Zeroing the platform

The zero is the platform pose with the HSAs at 0° and the plate sitting flat
(its weight is taken as negligible). To set it:

1. Enable the motors, `Set Zero Here` in `joint_commander_gui` (or
   `/hsa_odrive_control/zero_encoders`), command every joint to 0°.
2. Press **⌖ Set Platform Zero (camera)** in `joint_commander_gui`, or
   ```bash
   ros2 service call /set_platform_zero std_srvs/srv/Trigger
   ```
   The GUI button only enables while every HSA reads 0 ± 1° and the service
   exists; the service itself refuses unless `/hsa_odrive_control/joint_states`
   is fresh and all within `joints_zero_tol_deg` of 0 (`require_joints_zero:
   false` drops the guard), and unless a tag was seen within `max_age`.

The current camera → platform transform becomes the static TF
`camera → platform_zero` and is written to `zero_file`
(`~/.config/camera_tracking/platform_zero.yaml`), so it is reloaded on the
next start. Calling the service again overwrites it; delete the file to start
without one. Until a zero exists `/platform_pose` is silent (the node says so
at startup) but `/platform_pose_camera` and the TF `camera → platform` are
published, so the detector can be checked first.

Once zeroed, **`/platform_pose` is the platform expressed in `platform_zero`**:
identity at rest, and the plate's displacement (m) and rotation from rest as
the HSAs move. `/platform_zero_set` (latched) says whether a zero exists.

## Install (once)

```bash
sudo apt install ros-kilted-usb-cam ros-kilted-apriltag-ros ros-kilted-image-proc \
                 ros-kilted-cv-bridge ros-kilted-rqt-image-view ros-kilted-camera-calibration
cd ~/wrist_26/odrive_ros_ws          # or wherever odrive_ros_ws lives
colcon build --packages-select camera_tracking
source install/setup.bash
```

The baseplate camera is an Arducam OV9281 (global shutter, monochrome,
1280x800). It enumerated as `/dev/video8` with the laptop webcam and RealSense
also attached; check after every replug:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video8 --list-formats-ext     # MJPG: 120 fps at 1280x800 / 1280x720
```

`config/usb_cam.yaml` asks for 1280x800 MJPG at 120 fps: the full sensor,
which covers the perception spec's 720p at ≥ 100 Hz with 80 more rows. Those
rows matter: with the plate ~80 mm from the lens the 40 mm tag ring reaches
the top and bottom edges of the frame, and at 1280x720 the tag at 6 o'clock
was cut off. YUYV only reaches 10 fps. The sensor is monochrome but the only fast
path is MJPG decoded to rgb8; the detector converts to grey itself. Exposure is forced to manual by `camera.launch.py` through v4l2-ctl
(`exposure:=50` = 5 ms, in 100 us units; one frame at 120 fps is 8.3 ms),
because usb_cam's own exposure parameters do not reach this camera and its
default auto mode (15.7 ms) caps the rate near 60 fps. Add light rather than
exposure.

### Frame rates and where they are lost

The camera, rectify and AprilTag nodes are components of one multi-threaded
container with intra-process communication, so the detector sees every frame.
Each 1280x800 rgb8 frame is 3 MB; across process boundaries Fast DDS drops
most of them (measured on 2026-10-12, 120 fps source):

| Path | Rate |
|------|------|
| camera_info (small) to any process | ~119 Hz |
| image to another process, default transport | ~20 Hz |
| image to another process, `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` everywhere | ~70 Hz |
| detections, usb_cam in a separate process | ~57 Hz |

`FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` is **not** set by the launch files:
a node started with it receives nothing from nodes without it (tag_monitor lost
the ODrive topics that way), so it only works if every terminal exports it,
e.g. from `~/.bashrc`. Do that only if you need the external image rate
(bagging full-rate video); the detector does not need it.

```bash
ros2 topic hz /apriltag/detections         # the detector's real rate
ros2 topic hz /tracking_cam/camera_info     # the camera's real rate
```

`ros2 topic hz /tracking_cam/image_raw` measures the transport to your
terminal, not the camera. With everything in one container the detector ran at
~70 Hz at `detector.decimate: 1.0` and kept up with the camera at 2.0, which is
the default in `config/tags.yaml` (decimation only affects the quad search; the
corners are still refined on the full-resolution image). The 12 mm tags are
small: if they are missed at 2.0, drop to 1.0 and accept the lower rate.

Do not `ros2 component unload` nodes from the running container: with
intra-process communication it segfaults the whole container (camera included).
Restart the launch instead.

## Configure

| File | What to set |
|------|-------------|
| `config/usb_cam.yaml` | resolution / fps / pixel format of the camera |
| `config/camera_info.yaml` | intrinsics. Currently a placeholder (old Brio values at 1280x720); **recalibrate** for the OV9281 (below) |
| `config/tags.yaml` | AprilTagNode: `family: 16h5`, `size: 0.012`, `tag.ids: [0,1,2,3]`. 16h5 has few bits, so keep `max_hamming: 0` and the id filter |
| `config/platform.yaml` | `platform_pose`: tag layout, `tag_priority`, `pose_source`, `zero_file`, the zeroing guard |

Calibrate (print a checkerboard, e.g. 8x6 inner corners, 25 mm squares):

```bash
ros2 launch camera_tracking camera.launch.py
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    --ros-args -r image:=/tracking_cam/image_raw -r camera:=/tracking_cam
```

Click **Save** then **Commit** when the bars are green; or unpack
`/tmp/calibrationdata.tar.gz` and copy `ost.yaml` over `config/camera_info.yaml`
(keep `camera_name: tracking_cam`), then rebuild.

## Run

Full tracking stack (camera + apriltag + platform_pose + tag_monitor):

```bash
ros2 launch camera_tracking tracking.launch.py
```

Launch arguments: `video_device`, `platform_config` (default
`config/platform.yaml`), `tag_id` (the tag `tag_monitor` watches; -1 = any
tag), `output_dir` (default `~/wrist_captures`), `show_window` (true),
`image_topic` (`image_rect`, or `image_raw` to skip rectification),
`start_camera` (false when replaying a bag).

The rectify and AprilTag nodes run as components in one container
(`/tracking_cam/tracking_container`) with intra-process communication. Sending
1280x720 frames between processes drops samples and the detector then skips
frames (the tag flickers between detected and lost), so keep them together.

Piecewise, each in its own terminal:

```bash
ros2 launch camera_tracking camera.launch.py
ros2 launch camera_tracking apriltag.launch.py container:=/tracking_cam/tracking_container
ros2 run camera_tracking platform_pose --ros-args --params-file $(ros2 pkg prefix camera_tracking)/share/camera_tracking/config/platform.yaml
ros2 run camera_tracking tag_monitor  --ros-args -p tag_id:=0
```

Watch the result:

```bash
ros2 topic echo /platform_pose            # platform in platform_zero (after zeroing)
ros2 topic echo /platform_source_tag      # which tag anchors the pose (0 preferred)
ros2 topic echo /platform_pose/tag_consistency_mm
ros2 run tf2_tools view_frames            # camera → platform_zero → platform
```

Start the wrist stack as usual (`odrive_can wrist_launch.yaml` + `hsa_odrive_control`)
so the monitor has motor positions to record; without it the overlay shows `n/a`.

### Seeing the camera view

- `tag_monitor` opens an OpenCV window (`show_window:=true`) with the tag
  outline, DETECTED / LOST status, extension angles (deg), raw ODrive turns and
  the last commanded angles.
- The same view is published on `/tag_monitor/image_annotated`; view it with
  `ros2 run rqt_image_view rqt_image_view` (also shows `/tracking_cam/image_rect`).
- Keys in the window: `s` saves a snapshot, `q` quits.
- From a terminal: `ros2 service call /tag_monitor/capture std_srvs/srv/Trigger`

### Screenshots when the tag is not detected

`tag_monitor` declares the tag lost when the detector has reported a frame
without `tag_id` (any tag if -1) and none with it for `lost_after` (0.5 s).
A gap in detection messages alone (camera or detector stalled) is not a loss;
the overlay then shows `NO DETECTIONS MSGS` after `stale_after` (2 s). On the
transition to lost it saves immediately, then every `capture_interval` (2 s)
while still lost. By default it
only does this after the tag has been seen at least once
(`require_prior_detection`), so pointing the camera at nothing does not fill the
disk. Each capture writes into `output_dir`:

| File | Content |
|------|---------|
| `<stamp>_lost.png` | annotated frame (status + motor positions burned in) |
| `<stamp>_lost_raw.png` | the frame as received (`save_raw`) |
| `<stamp>_lost.yaml` | timestamp, reason, detected ids, seconds since last seen, `joint_states_deg`, `odrive_positions_turns`, `joint_commands_deg` |
| `captures.csv` | one row per capture, same fields, appended |

Manual captures use `_manual` instead of `_lost`.

Parameters: `tag_id`, `lost_after`, `stale_after`, `capture_on_lost`, `capture_interval`,
`require_prior_detection`, `output_dir`, `save_raw`, `show_window`,
`publish_annotated`, and the topic names `image_topic`, `detections_topic`,
`joint_states_topic`, `odrive_positions_topic`, `joint_commands_topic`.

## Topics

| Topic | Type | From |
|-------|------|------|
| `/tracking_cam/image_raw`, `/camera_info` | sensor_msgs | usb_cam |
| `/tracking_cam/image_rect` | sensor_msgs/Image | image_proc rectify |
| `/apriltag/detections` | apriltag_msgs/AprilTagDetectionArray | AprilTagNode (ids, corners; no pose) |
| TF `camera -> tag16h5:<id>` | tf2 | AprilTagNode's own per-tag pose (debugging only) |
| `/platform_pose` | geometry_msgs/PoseStamped | platform_pose: platform in `platform_zero`, once zeroed, while a tag is visible |
| `/platform_pose_camera` | geometry_msgs/PoseStamped | platform_pose: platform in `camera`, always while visible |
| `/platform_tag_visible` | std_msgs/Bool | platform_pose: a layout tag was in the latest frame |
| `/platform_source_tag` | std_msgs/Int32 | platform_pose: id anchoring the pose, -1 = none |
| `/platform_zero_set` | std_msgs/Bool (latched) | platform_pose |
| `/platform_pose/tag_consistency_mm` | std_msgs/Float32MultiArray | platform_pose, when > 1 tag visible |
| TF `camera -> platform_zero` (static), `platform_zero -> platform` | tf2 | platform_pose (`camera -> platform` until zeroed) |
| `/set_platform_zero` | std_srvs/Trigger (service) | platform_pose |
| `/plate_mount_ok`, `/plate_mount_status` | std_msgs/Bool, String | plate_guard, 2 Hz |
| `/platform_tag_detected` | std_msgs/Bool | tag_monitor |
| `/tag_monitor/image_annotated` | sensor_msgs/Image | tag_monitor |

`plate_guard` parameters (`config/platform.yaml`): `expected_id1_bearing_deg`,
`tolerance_deg` (15), `stale_after` (1 s), `warn_period` (5 s), `rate` (2 Hz),
`detections_topic` and the tag layout.

`plate_check` parameters: `drive` (false), `wiggle_deg` (20), `settle_s` (2),
`min_move_mm` (2), `pairs` (flat list, default 0 1 2 3 4 5), `report_period`
(1 s), the tag layout parameters and the three topic names.

`platform_pose` parameters (`config/platform.yaml`): `detections_topic`,
`camera_info_topic`, `joint_states_topic`, `camera_frame`, `platform_frame`,
`zero_frame`, `tag_size`, `tag_ids`, `tag_x_mm`, `tag_y_mm`, `tag_priority`,
`pose_source` (priority | all), `max_age` (0.1 s, freshness required to zero),
`stale_after` (1 s), `zero_file`, `require_joints_zero`, `joints_zero_tol_deg`
(1°), `joints_max_age` (1 s), `publish_consistency`.

The maths (layout, PnP, hierarchy, zero) is in `camera_tracking/platform_geometry.py`
and covered by `test/test_platform_geometry.py` (`python3 -m pytest test`).

## Troubleshooting

- **No image**: `ros2 topic hz /tracking_cam/image_raw`; wrong `video_device`,
  or the camera does not support `mjpeg2rgb` at 1280x720 — check
  `v4l2-ctl -d /dev/videoN --list-formats-ext` and edit `usb_cam.yaml`.
- **Image but no detections**: `ros2 topic echo /apriltag/detections`. Tag too
  small/blurred: set `detector.decimate: 1.0`, turn off autofocus, more light.
  Tag ids filtered out by `tag.ids` in `tags.yaml`.
- **Detections but no `/platform_pose`**: no zero set yet (check
  `/platform_zero_set`; `/platform_pose_camera` still works), the detected ids
  are not in `tag_ids`, or no `camera_info` (the node warns "no camera_info
  yet"; `Calibration file not found` from usb_cam is the usual cause,
  `camera name mismatch` warnings are harmless).
- **`/set_platform_zero` refused**: the message says why: joint states missing
  or not at 0°, or the tag not seen within `max_age`.
- **Phantom tags**: 16h5 false positives show up as extra ids in
  `/apriltag/detections`; the id filter in `tags.yaml` and `max_hamming: 0`
  keep them out of the pose. A phantom with a real id is caught by
  `tag_consistency_mm` jumping.
- **Fallback pose jumps when ID0 disappears**: the outer tag's offset or
  printed orientation differs from `platform.yaml`; read
  `tag_consistency_mm` with all tags visible and fix the numbers.
- **Window never opens**: no `$DISPLAY` (ssh) — use `show_window:=false` and
  `rqt_image_view` on `/tag_monitor/image_annotated`.
