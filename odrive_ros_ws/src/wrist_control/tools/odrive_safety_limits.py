#!/usr/bin/env python3
"""
odrive_safety_limits.py: write the wrist's hardware safety limits into each
ODrive's flash over USB, using the `odrive` Python package (the same library
odrivetool uses).

The limits set here are enforced by the ODrive firmware itself, every control
cycle, whether or not ROS / the PC / the CAN bus is running:

  1. Temperature: FET (inverter) thermistor limit, default upper = 80 °C.
     If a motor thermistor is enabled, its limits are set the same.
  2. Instantaneous current: 3.5 A, on either the DC bus or the motor phase
     current (you choose with --current, see below).

The 1-minute (<3 A) and 5-minute (<2 A) average-current limits can NOT be done
in firmware; they need the host-side watchdog node (not part of this script).

Requirements
------------
  * ODrive Pro / S1 on firmware 0.6.x (the script refuses anything else).
  * The `odrive` package that matches that firmware:  pip install odrive
  * The ODrive connected over USB and powered from its DC supply.
  * Every axis in IDLE. Disable the wrist first (joint_commander_gui
    "Disable", or `ros2 service call /hsa_odrive_control/disable
    std_srvs/srv/Trigger`). The script refuses to touch an axis that isn't
    IDLE. ROS and the CAN nodes may keep running; USB and CAN can coexist.

Usage
-----
Run from this directory (it is not installed by colcon):

  # 1. Look first: print the current limits and temperatures, change nothing.
  python3 odrive_safety_limits.py --check

  # 2a. Apply, limiting DC bus current (what joint_commander_gui displays):
  python3 odrive_safety_limits.py --current bus

  # 2b. ...or apply, limiting motor phase current instead:
  python3 odrive_safety_limits.py --current phase --soft-max 3.0

  # 3. The ODrive saves and reboots. Re-run --check to confirm the values
  #    survived the reboot.
  python3 odrive_safety_limits.py --check

The wrist has six ODrives. Either plug them in one at a time and run the
script for each, or connect them all through a USB hub and pass --count 6 so
it waits for all six and configures each.

What each --current mode writes
-------------------------------
  bus:    config.dc_max_positive_current = --max-current
          Board-level limit on current drawn from the supply. Exceeding it
          raises DC_BUS_OVER_CURRENT and the motor is turned off.
  phase:  axis0.config.motor.current_hard_max = --max-current
          axis0.config.motor.current_soft_max = --soft-max
          hard_max is the trip level (CURRENT_LIMIT_VIOLATION). soft_max is
          the most current the controller will ever *command*, so it must be
          below hard_max, or normal moves can trip the fault. Lowering it also
          lowers the motor's maximum torque.

Temperature limits
------------------
  config.inverter0.temp_limit_lower = --temp-lower  (default 75 °C)
  config.inverter0.temp_limit_upper = --temp-upper  (default 80 °C)

  Between lower and upper the firmware ramps the current limit down linearly;
  at upper it reaches 0 A and an over-temperature error (INVERTER_OVER_TEMP /
  MOTOR_OVER_TEMP) is raised. Factory defaults on Pro/S1 are 84 / 103 °C.
  Lowering --temp-lower makes the derating start earlier.

  NOTE: the ODrive docs disagree about whether config.inverter0.* is writable
  outside developer mode. The script reads every value back after writing it.
  If the firmware ignored a write, it reports it as FAILED. Then that limit
  must be covered by the host-side watchdog instead.

After the run, check what happened with `--check`, and if an axis faults, see
why in odrivetool with:
  odrv0.axis0.active_errors, odrv0.axis0.disarm_reason
"""

import argparse
import math
import sys

try:
    import odrive
except ImportError:
    sys.exit("The 'odrive' package is not installed. Run:  pip install odrive")

AXIS_STATE_IDLE = 1

FET_TEMP_PATH = "axis0.motor.fet_thermistor.temperature"
MOTOR_TEMP_PATH = "axis0.motor.motor_thermistor.temperature"
MOTOR_THERMISTOR_ENABLED_PATH = "axis0.motor.motor_thermistor.config.enabled"

# Everything --check prints, whether or not this run writes it.
REPORT_PATHS = [
    "config.dc_max_positive_current",
    "axis0.config.motor.current_soft_max",
    "axis0.config.motor.current_hard_max",
    "config.inverter0.temp_limit_lower",
    "config.inverter0.temp_limit_upper",
    MOTOR_THERMISTOR_ENABLED_PATH,
    "axis0.motor.motor_thermistor.config.temp_limit_lower",
    "axis0.motor.motor_thermistor.config.temp_limit_upper",
    FET_TEMP_PATH,
    MOTOR_TEMP_PATH,
]

_MISSING = object()


def get_path(obj, path):
    """Read a dotted attribute path; returns _MISSING if any part is absent."""
    for part in path.split("."):
        try:
            obj = getattr(obj, part)
        except AttributeError:
            return _MISSING
    return obj


def set_path(obj, path, value):
    parent_path, _, name = path.rpartition(".")
    parent = get_path(obj, parent_path) if parent_path else obj
    if parent is _MISSING:
        raise AttributeError(f"{parent_path} not found")
    setattr(parent, name, value)


def fmt(value):
    if value is _MISSING:
        return "(not present on this firmware)"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def serial_str(odrv):
    return format(odrv.serial_number, "X")


def desired_settings(args, odrv):
    """(path, value) pairs to write for this ODrive."""
    settings = []
    if args.current == "bus":
        settings.append(("config.dc_max_positive_current", args.max_current))
    else:
        # soft before hard so the soft limit is never above the hard limit.
        settings.append(("axis0.config.motor.current_soft_max", args.soft_max))
        settings.append(("axis0.config.motor.current_hard_max", args.max_current))

    settings.append(("config.inverter0.temp_limit_lower", args.temp_lower))
    settings.append(("config.inverter0.temp_limit_upper", args.temp_upper))

    if get_path(odrv, MOTOR_THERMISTOR_ENABLED_PATH) is True:
        settings.append(("axis0.motor.motor_thermistor.config.temp_limit_lower", args.temp_lower))
        settings.append(("axis0.motor.motor_thermistor.config.temp_limit_upper", args.temp_upper))
    return settings


def report(odrv):
    for path in REPORT_PATHS:
        print(f"    {path:<55} {fmt(get_path(odrv, path))}")


def check_firmware(odrv):
    major, minor = odrv.fw_version_major, odrv.fw_version_minor
    version = f"{major}.{minor}.{odrv.fw_version_revision}"
    if (major, minor) < (0, 6):
        return version, False
    return version, True


def apply(args, odrv):
    """Write, read back, and save. Returns True if every setting applied."""
    state = get_path(odrv, "axis0.current_state")
    if state != AXIS_STATE_IDLE:
        print(f"  axis0 is in state {state}, not IDLE (1). Disable the wrist first. Skipped.")
        return False

    all_ok = True
    applied_any = False
    for path, value in desired_settings(args, odrv):
        before = get_path(odrv, path)
        if before is _MISSING:
            print(f"  FAILED  {path}: not present on this firmware")
            all_ok = False
            continue
        try:
            set_path(odrv, path, value)
        except Exception as e:  # read-only endpoint, out of range, etc.
            print(f"  FAILED  {path}: write raised {type(e).__name__}: {e}")
            all_ok = False
            continue
        after = get_path(odrv, path)
        if isinstance(after, float) and math.isclose(after, value, rel_tol=1e-4, abs_tol=1e-4):
            print(f"  ok      {path}: {fmt(before)} -> {fmt(after)}")
            applied_any = True
        else:
            print(f"  FAILED  {path}: wrote {value}, read back {fmt(after)} "
                  "(firmware ignored the write?)")
            all_ok = False

    if args.no_save:
        print("  --no-save given: values are live but NOT saved; lost on reboot.")
    elif applied_any:
        print("  Saving to flash; the ODrive will reboot and disconnect...")
        try:
            odrv.save_configuration()
        except Exception:
            pass  # save reboots the board, so losing the connection is expected
        print("  Saved. Re-run with --check once it has rebooted (~5 s).")
    else:
        print("  Nothing applied, so nothing saved.")
    return all_ok


def parse_args():
    p = argparse.ArgumentParser(
        description="Write wrist safety limits to ODrive flash over USB. "
                    "See the module docstring for full usage.")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true",
                      help="only print current limits and temperatures; change nothing")
    mode.add_argument("--current", choices=["bus", "phase"],
                      help="which current to limit: DC bus or motor phase")
    p.add_argument("--max-current", type=float, default=3.5,
                   help="instantaneous current trip level, A (default 3.5)")
    p.add_argument("--soft-max", type=float, default=3.0,
                   help="phase mode only: max commanded current, A; must be < --max-current (default 3.0)")
    p.add_argument("--temp-lower", type=float, default=75.0,
                   help="temperature where current derating starts, °C (default 75)")
    p.add_argument("--temp-upper", type=float, default=80.0,
                   help="temperature where current reaches 0 and the axis faults, °C (default 80)")
    p.add_argument("--count", type=int, default=1,
                   help="number of ODrives to wait for on USB (default 1)")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="seconds to wait for the ODrive(s) to appear (default 10)")
    p.add_argument("--no-save", action="store_true",
                   help="write the values but don't save to flash (for testing)")
    args = p.parse_args()

    if args.temp_lower >= args.temp_upper:
        p.error("--temp-lower must be below --temp-upper")
    if args.current == "phase" and args.soft_max >= args.max_current:
        p.error("--soft-max must be below --max-current, or normal moves can trip the fault")
    return args


def main():
    args = parse_args()

    print(f"Waiting up to {args.timeout:.0f} s for {args.count} ODrive(s) on USB...")
    try:
        found = odrive.find_any(count=args.count, timeout=args.timeout)
    except Exception as e:
        sys.exit(f"Could not find {args.count} ODrive(s): {type(e).__name__}: {e}")
    devices = found if isinstance(found, (list, tuple)) else [found]

    all_ok = True
    for odrv in devices:
        version, supported = check_firmware(odrv)
        print(f"\nODrive {serial_str(odrv)}  firmware {version}")
        if not supported:
            print("  Firmware older than 0.6: these setting names don't apply. Skipped.")
            all_ok = False
            continue
        if args.check:
            report(odrv)
        else:
            all_ok &= apply(args, odrv)

    if not all_ok:
        print("\nSome limits were NOT applied (see FAILED / Skipped above).")
        sys.exit(1)


if __name__ == "__main__":
    main()
