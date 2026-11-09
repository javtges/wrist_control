# Data-driven kinematic model of the wrist (base flat on the table)

Reference: Tedrake, *Underactuated Robotics*, ch. 18 "System Identification"
(<https://underactuated.mit.edu/sysid.html>). Section names below are from that chapter.

The HSAs have no usable first-principles forward kinematics: the rigid-leg Stewart model in
`stewart_kinematics.py` predicts 5.2 mm rise for all-six +20 deg where the plate rises 1.3 mm
(`camera_tracking/doc/camera_tracking.md`, "Measured on the wrist"). So we identify the map
`leg angles -> plate pose` from data, the way soft_stewart did, but with a linear-in-parameters
model first so identifiability and excitation can be checked (ch. 18 "Parameter identification").

## What is being identified

| | static (kinematic) | dynamic |
|---|---|---|
| model | `y = W(q) alpha`, polynomial library in the 6 leg angles, optional hysteresis term | ARX `y[n+1] = sum A_i y[n-i] + sum B_i u[n-i]` |
| chapter | "Residual physics models with linear function approximators", "Experiment design" | "Autoregressive models", "Equation error vs simulation error" |
| input q / u | measured extension deg (`/hsa/wrist/state.extension_deg`) | commanded deg |
| output y | `[x_mm, y_mm, z_mm, tilt_x_deg, tilt_y_deg, twist_deg]` from `platform_geometry.plate_metrics` | same |
| data file | `*_points.csv` (one row per settled setpoint) | `*_samples.csv` (every pose frame) |

## Steps

1. **Bring-up** (`doc/running.md`): CAN at 500 kbit with `txqueuelen 1000`, then
   `ros2 launch wrist_control wrist_control.launch.py camera:=true`, `~/enable`, `~/zero_encoders`,
   command all 0 deg, confirm `/plate_mount_ok`, call `/set_platform_zero`. The base must not move
   relative to the camera for the whole session: the pose is in the `platform_zero` frame.
   Then turn **PUBLISHING off** in joint_commander_gui (and close spacemouse_commander). Two
   publishers on `/hsa_odrive_control/joint_commands` interleave setpoints and the motors chatter
   between them; on 2026-11-08 the GUI held 0 deg at 100 Hz while a sweep ramped to 30 deg and all six
   vibrated. `sysid_excite` now waits while anything else is publishing commands, and aborts without
   commanding if another publisher starts mid-run. Keeping the GUI open with publishing off is fine.
2. **Pick the range.** Default `max_deg:=120` keeps to the near-linear region (~0.07 mm/deg all-six,
   knee after 120 deg). Widen to 180-360 only after the 120 deg fit is healthy; the knee needs
   `--degree 3` or more rows per level.
3. **Excite and record.** Three static runs and one dynamic run:
   ```bash
   ros2 run wrist_control sysid_excite --ros-args -p mode:=static -p pattern:=grid -p grid_levels:=4 -p n_points:=300
   ros2 run wrist_control sysid_excite --ros-args -p mode:=static -p pattern:=random -p n_points:=200
   ros2 run wrist_control sysid_excite --ros-args -p mode:=static -p pattern:=sweep -p grid_levels:=7
   ros2 run wrist_control sysid_excite --ros-args -p mode:=dynamic -p pattern:=multisine -p duration_s:=600
   ```
   `grid`/`random` return to 0 between points (soft_stewart's zero-interleave) so each sample has
   the same HSA history; `sweep` does monotone staircases so hysteresis is visible. Each static
   point is held `settle_s`=3 s then averaged for `avg_s`=0.5 s; samples are dropped unless the tag
   is visible and the mount check passes. Commands are slew-limited (`rate_deg_s`) and the node
   returns to 0 on exit. Ctrl-C slews back to 0 at `rate_deg_s`; a second Ctrl-C stops and holds the
   current setpoint. Files land in `~/.local/share/wrist_sysid/`.
4. **Filter tag-set switches.** When a tag leaves or enters the camera frame the plate pose jumps
   (1-7 deg tilt, up to 2.5 mm) in one frame and can stay shifted. `sysid_filter` finds those
   jumps in the samples log and flags points that flicker during the average or sit between the
   first and last switch of an excursion from 0 deg:
   ```bash
   ros2 run wrist_control sysid_filter ~/.local/share/wrist_sysid/sysid_static_grid_<stamp>_points.csv
   # -> <stamp>_points_flagged.csv (all rows + pose_ok, pose_flag), <stamp>_points_clean.csv
   ```
   Fit on the `_points_clean.csv` files. On the 2026-11-08 sweep it kept 272 of 325 points; the two
   runs then agree to 0.67 deg / 0.31 mm, against 3.2 deg before.
5. **Fit.**
   ```bash
   ros2 run wrist_control sysid_fit ~/.local/share/wrist_sysid/sysid_static_grid_*_points.csv \
       ~/.local/share/wrist_sysid/sysid_static_random_*_points.csv --degree 2 --sparse 0.02 -v --plot --out wrist_kin.npz
   ros2 run wrist_control sysid_fit ~/.local/share/wrist_sysid/sysid_static_sweep_*_points.csv --hysteresis -v
   ros2 run wrist_control sysid_fit ~/.local/share/wrist_sysid/sysid_dynamic_multisine_*_samples.csv --arx --na 2 --nb 2 --dt 0.05 --plot
   ```
6. **Read the diagnostics** before trusting anything:
   - `cond(W)` and the singular values: a tiny trailing singular value means a direction of parameter
     space the data never excited; the fitter prints which terms span it. Add levels or points, do
     not add basis terms (ch. 18 "Experiment design", "Identifiability").
   - train vs test RMSE per output, split by whole points. Test RMSE near the 0.02 mm / 0.1 deg rest
     repeatability means the model is at the sensor floor; test much worse than train is overfitting.
   - `d z_mm / d q_i at 0` should be ~0.012 mm/deg per leg and similar across legs.
   - ARX: compare equation-error RMSE to simulation-error (rollout) RMSE. Only the rollout number
     says whether the model is usable for prediction (ch. 18 "Equation error vs simulation error").
7. **Use.** `wrist_control.sysid_fit.WristKinematicModel('wrist_kin.npz')(theta_deg)` returns the 6
   pose numbers; invert numerically for IK, or fit the IK direction the same way with inputs and
   outputs swapped (as soft_stewart's `ik_model`).

## Pose-vs-angle table (sweep)

`pattern:=sweep` alone is the plate-pose-per-angle table: staircases up then down on all six
together, each leg alone, and each adjacent pair (13 sets x `2*grid_levels-1` setpoints).
Each `*_points.csv` row is one settled angle: `x_mm`, `y_mm`, `lateral_mm` (=hypot of the mean
x/y), `z_mm`, `tilt_deg`, `tilt_dir_deg`, `twist_deg`, the std columns, then per motor `cmd{i}`
(commanded deg), `meas{i}` (measured extension deg) and `motor_turns{i}` (raw ODrive
`pos_estimate`, without the zero offset). `tag_mask` (bit i = layout tag i in the fit), `tag_mask_changes` (switches during the
averaging window) and `fit_rms_px` say which tags the pose came from; drop or flag points where the
mask changed or the RMS is high (see camera_tracking.md, "Tilt depends on which tags are seen").

```bash
ros2 run wrist_control sysid_excite --ros-args -p mode:=static -p pattern:=sweep \
    -p min_deg:=0.0 -p max_deg:=120.0 -p grid_levels:=7    # 20 deg steps, 169 setpoints, ~10 min
```

## How this maps onto ch. 18

- *Linear in parameters.* The chapter's inverse-dynamics regressor `W(q, qdot, qddot) alpha` becomes
  a kinematic regressor `W(q) alpha`: the nonlinearity is in the state (`q_i q_j`, …), never in the
  parameters, so ordinary least squares applies and the SVD of `W` tells you what is identifiable.
- *Basis library + sparsity.* The polynomial library plus sequential thresholded least squares
  (`--sparse`) is the "residual physics with linear function approximators" recipe.
- *Hysteresis as sign-dependent friction.* The chapter adds `sign(qdot)` friction columns; here the
  up/down direction column `h` plays the same role, and it is only identifiable from `sweep` data.
- *Experiment design.* `grid` sweeps corners, `random` fills the interior, `multisine` with
  leg-distinct phases gives persistent excitation for the ARX fit. The condition number of `W` is
  the number to minimise when choosing levels and ranges.
- *Simulation error.* `arx_rollout` scores the dynamic model the way the chapter recommends.
- *Data alignment.* `/platform_pose` carries the image stamp, the ODrive feed has no header, so the
  recorder joins on receive time and the ARX fit resamples onto a uniform grid (the chapter's
  warning about differentiating noisy signals is why only positions are regressed).

## Relation to soft_stewart

`soft_stewart/stewart_ros/src/maxdiff_ros/maxdiff_ros/workspace.py` is the same experiment:
a shuffled 4-level 6-D grid of servo angles, zeros interleaved, 3 s settle, `(angles, [xyz,rpy])`
rows saved to `.npy`. `rosbag_kinematic_model.ipynb` then trains two 6-128-6 ReLU MLPs (FK and IK)
with an 80/20 random split, and `fk_eval.py` / `ik_eval.py` serve the TorchScript traces and test
IK on the hardware against the recorded poses and a rigid-Stewart approximation. Differences here:
the fit is linear in parameters first (so cond(W) and identifiability are inspectable and the
model is a few numbers, not a network), inputs are scaled (soft_stewart never normalised degrees vs
metres), the split is by point not by row, hysteresis is measured instead of reset away, and a
dynamic ARX model is fitted from the same recorder. The MLP is a reasonable second step once the
polynomial fit plateaus above the sensor floor.
