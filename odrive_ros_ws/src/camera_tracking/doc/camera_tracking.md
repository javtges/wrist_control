# camera_tracking — baseplate camera + AprilTag platform tracking

A USB camera on the baseplate looks up at an AprilTag fixed to the moving
platform. This package drives the camera (`usb_cam`), undistorts the image
(`image_proc`), detects the tag (`apriltag_ros`), publishes the platform pose
and runs a monitor that shows the camera view with the motor positions drawn on
it and saves screenshots + motor positions whenever the tag is lost.

```
usb_cam ──► /tracking_cam/image_raw ──► rectify ──► /tracking_cam/image_rect ──► AprilTagNode
                                                                                  │
                                       TF camera→tag36h11:<id>  ◄─────────────────┤
                                       /apriltag/detections     ◄─────────────────┘
                 platform_pose ◄── TF                 tag_monitor ◄── detections + image_rect
                 /platform_pose                        + /hsa_odrive_control/joint_states
                 /platform_tag_visible                 + /odrive_positions
                                                       + /hsa_odrive_control/joint_commands
                                                       ──► ~/wrist_captures/*.png,*.yaml,captures.csv
```

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

`config/usb_cam.yaml` asks for 1280x800 MJPG at 120 fps. YUYV only reaches
10 fps. Exposure is forced to manual by `camera.launch.py` through v4l2-ctl
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
pose is still refined on the full-resolution image).

Do not `ros2 component unload` nodes from the running container: with
intra-process communication it segfaults the whole container (camera included).
Restart the launch instead.

## Configure

| File | What to set |
|------|-------------|
| `config/usb_cam.yaml` | resolution / fps / pixel format of the camera |
| `config/camera_info.yaml` | intrinsics. Currently a placeholder (old Brio values at 1280x800); **recalibrate** for the OV9281 (below) |
| `config/tags.yaml` | `size` = tag edge length in metres (black border to border). Optionally `tag.ids`/`frames` to restrict to the platform tag |

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
ros2 launch camera_tracking tracking.launch.py tag_id:=0
```

Launch arguments: `video_device`, `tag_id` (-1 = any tag), `output_dir`
(default `~/wrist_captures`), `show_window` (true), `image_topic`
(`image_rect`, or `image_raw` to skip rectification), `start_camera`
(false when replaying a bag).

The rectify and AprilTag nodes run as components in one container
(`/tracking_cam/tracking_container`) with intra-process communication. Sending
1280x720 frames between processes drops samples and the detector then skips
frames (the tag flickers between detected and lost), so keep them together.

Piecewise, each in its own terminal:

```bash
ros2 launch camera_tracking camera.launch.py
ros2 launch camera_tracking apriltag.launch.py container:=/tracking_cam/tracking_container
ros2 run camera_tracking platform_pose --ros-args -p tag_id:=0
ros2 run camera_tracking tag_monitor  --ros-args -p tag_id:=0
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
| TF `camera -> tag36h11:<id>` | tf2 | AprilTagNode (the pose) |
| `/platform_pose` | geometry_msgs/PoseStamped | platform_pose, only while the TF is fresh |
| `/platform_tag_visible` | std_msgs/Bool | platform_pose |
| `/platform_tag_detected` | std_msgs/Bool | tag_monitor |
| `/tag_monitor/image_annotated` | sensor_msgs/Image | tag_monitor |

`platform_pose` parameters: `camera_frame` (camera), `tag_id` (0) or `tag_frame`
(explicit TF name, e.g. `platform_tag` if set in `tags.yaml`), `rate` (30 Hz),
`max_age` (1.0 s).

## Troubleshooting

- **No image**: `ros2 topic hz /tracking_cam/image_raw`; wrong `video_device`,
  or the camera does not support `mjpeg2rgb` at 1280x720 — check
  `v4l2-ctl -d /dev/videoN --list-formats-ext` and edit `usb_cam.yaml`.
- **Image but no detections**: `ros2 topic echo /apriltag/detections`. Tag too
  small/blurred: set `detector.decimate: 1.0`, turn off autofocus, more light.
  Tag ids filtered out by `tag.ids` in `tags.yaml`.
- **Detections but no `/platform_pose`**: `tag_id` does not match the detected
  id (`ros2 run tf2_tools view_frames` shows the frame names), or
  `camera_info.yaml` is missing so AprilTagNode cannot estimate a pose
  (`camera name mismatch` warnings are harmless, `Calibration file not found` is not).
- **Window never opens**: no `$DISPLAY` (ssh) — use `show_window:=false` and
  `rqt_image_view` on `/tag_monitor/image_annotated`.
