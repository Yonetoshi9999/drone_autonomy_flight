#!/usr/bin/env python3
"""
Mode 99 Companion Computer Script
Connects to ArduPilot SITL and runs the state machine:
  GUIDED arm → takeoff → Mode 99 → hover → land

Command rate: 20Hz (as expected by Mode 99)
"""

import sys
import time
import math
from pymavlink import mavutil

CONNECTION = 'tcp:127.0.0.1:5760'
CMD_HZ = 20       # Hz — Mode 99 expects commands at this rate
CMD_DT = 1.0 / CMD_HZ

def drain(master, duration=0.0):
    """Drain messages for `duration` seconds, printing STATUSTEXT."""
    deadline = time.time() + duration
    while True:
        remaining = deadline - time.time() if duration > 0 else 0.0
        if duration > 0 and remaining <= 0:
            break
        msg = master.recv_match(blocking=True, timeout=min(0.1, remaining) if duration > 0 else 0.05)
        if msg is None:
            if duration <= 0:
                break
            continue  # timeout but deadline not yet reached
        if msg.get_type() == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")

def wait_for_mode(master, mode_num, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg and msg.custom_mode == mode_num:
            return True
    return False

def send_ned_command(master, pos_n=0.0, pos_e=0.0, pos_d=0.0,
                     vel_n=0.0, vel_e=0.0, vel_d=0.0,
                     yaw=0.0, yaw_rate=0.0):
    """Send SET_POSITION_TARGET_LOCAL_NED to Mode 99 at 20Hz."""
    master.mav.set_position_target_local_ned_send(
        int(AP_HAL_millis()),             # time_boot_ms (approx)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_0101_1100_0000,            # type_mask: ignore acc(bits6-8) + yaw(bit10); use pos+vel+yaw_rate
        pos_n, pos_e, pos_d,              # position (m, NED)
        vel_n, vel_e, vel_d,              # velocity (m/s, NED)
        0.0, 0.0, 0.0,                    # acceleration (ignored)
        yaw, yaw_rate,                    # yaw (rad), yaw_rate (rad/s)
    )

_t0 = time.time()
def AP_HAL_millis():
    return int((time.time() - _t0) * 1000)

def get_position(master):
    """Return the LATEST LOCAL_POSITION_NED by draining stale buffered messages."""
    latest = None
    # Drain all queued LOCAL_POSITION_NED messages (non-blocking) to get the freshest one
    while True:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg is None:
            break
        latest = msg
    if latest:
        return latest.x, latest.y, latest.z, latest.vx, latest.vy, latest.vz
    # Nothing in buffer — do one blocking read
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
    if msg:
        return msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
    return None

_DIAG_KEYS = ('LQI_Thrust', 'EKF_pD', 'REF_pD', 'ERR_pD', 'ERR_vD', 'THR_out')

def run_at_20hz(master, duration_s, pos_n=0.0, pos_e=0.0, pos_d=0.0,
                vel_n=0.0, vel_e=0.0, vel_d=0.0,
                yaw=0.0, yaw_rate=0.0, label="",
                pos_d_start=None):
    """Send commands at 20Hz for `duration_s` seconds, printing state at 1Hz.

    pos_d_start: if set, ramp pos_d from pos_d_start toward pos_d at vel_d rate.
    This prevents a step-change in position reference that would saturate thrust.
    """
    t_start = time.time()
    last_print = 0.0
    cmd_count = 0
    nv = {}  # latest NAMED_VALUE_FLOAT snapshot
    # Ramp state: pos_d_cmd starts at pos_d_start (or pos_d if no ramp)
    pos_d_cmd = pos_d_start if pos_d_start is not None else pos_d

    while time.time() - t_start < duration_s:
        loop_t = time.time()

        # Ramp pos_d toward target at vel_d rate (prevents thrust saturation on step change)
        if pos_d_start is not None and vel_d != 0.0:
            pos_d_cmd += vel_d * CMD_DT  # step by vel_d * nominal dt each tick
            if vel_d < 0:  # climbing: NED pos_d decreasing toward negative target
                pos_d_cmd = max(pos_d, pos_d_cmd)
            else:           # descending: NED pos_d increasing toward positive target
                pos_d_cmd = min(pos_d, pos_d_cmd)
        else:
            pos_d_cmd = pos_d
        # Feed-forward velocity only while still ramping; zero once at target
        vel_d_active = vel_d if abs(pos_d_cmd - pos_d) > 0.01 else 0.0

        send_ned_command(master,
                         pos_n=pos_n, pos_e=pos_e, pos_d=pos_d_cmd,
                         vel_n=vel_n, vel_e=vel_e, vel_d=vel_d_active,
                         yaw=yaw, yaw_rate=yaw_rate)
        cmd_count += 1

        # Print state at 1Hz
        elapsed = time.time() - t_start
        if elapsed - last_print >= 1.0:
            last_print = elapsed
            p = get_position(master)
            alt_str = f"alt={-p[2]:.2f}m" if p else "alt=?"
            tgt_str = f"tgt_d={pos_d_cmd:.2f}" if pos_d_start is not None else ""
            diag = " ".join(f"{k}={nv[k]:.3f}" for k in _DIAG_KEYS if k in nv)
            print(f"  [{label} {elapsed:.0f}s] {alt_str} {tgt_str} cmds_sent={cmd_count}  {diag}")

        # Drain one message per tick: capture STATUSTEXT and NAMED_VALUE_FLOAT
        msg = master.recv_match(blocking=False)
        if msg:
            mt = msg.get_type()
            if mt == 'STATUSTEXT':
                print(f"  [FC] {msg.text.strip()}")
            elif mt == 'NAMED_VALUE_FLOAT':
                name = msg.name.rstrip('\x00')
                if name in _DIAG_KEYS:
                    nv[name] = msg.value

        # Sleep to maintain 20Hz
        elapsed_loop = time.time() - loop_t
        sleep_t = CMD_DT - elapsed_loop
        if sleep_t > 0:
            time.sleep(sleep_t)

    print(f"  [{label}] done: {cmd_count} commands sent in {duration_s:.0f}s ({cmd_count/duration_s:.1f}Hz)")

def main():
    print("=" * 70)
    print("MODE 99 COMPANION COMPUTER - 20Hz command loop")
    print("=" * 70)

    # ── 1. Connect ────────────────────────────────────────────────────────
    print(f"\n[1] Connecting to SITL at {CONNECTION} ...")
    master = mavutil.mavlink_connection(CONNECTION, source_system=255)
    hb = master.wait_heartbeat(timeout=15)
    if not hb:
        print("  ERROR: no heartbeat"); return 1
    print(f"  Connected. sysid={master.target_system}")

    # Request position and extended status data streams so LOCAL_POSITION_NED flows
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)   # LOCAL_POSITION_NED @ 10Hz
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)  # GPS_RAW_INT etc @ 5Hz

    # Wait for GPS fix AND EKF convergence (up to 40s)
    # MAVLink EKF_STATUS_FLAGS (EKF_STATUS_REPORT.flags):
    #   0x002 EKF_VELOCITY_HORIZ      - horizontal velocity estimate OK
    #   0x010 EKF_POS_HORIZ_ABS       - absolute horizontal position OK (HORIZ_POS_ABS)
    #   0x200 EKF_PRED_POS_HORIZ_ABS  - predicted absolute horiz position (PRED_HORIZ_POS_ABS)
    # ArduCopter position_ok() when DISARMED: accepts PRED_HORIZ_POS_ABS (0x200) OR HORIZ_POS_ABS (0x010)
    # ArduCopter position_ok() when ARMED: requires HORIZ_POS_ABS (0x010), no CONST_POS_MODE (0x080)
    print("  Waiting for GPS fix + EKF convergence (up to 40s)...")
    ekf_deadline = time.time() + 40.0
    gps_ok = False
    ekf_pos_ok = False
    while time.time() < ekf_deadline:
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == 'GPS_RAW_INT' and msg.fix_type >= 3 and msg.satellites_visible >= 6:
            if not gps_ok:
                print(f"  GPS fix OK: fix_type={msg.fix_type} sats={msg.satellites_visible}")
                gps_ok = True
        elif mt == 'EKF_STATUS_REPORT':
            has_vel      = bool(msg.flags & 0x002)   # EKF_VELOCITY_HORIZ
            has_pos_abs  = bool(msg.flags & 0x010)   # EKF_POS_HORIZ_ABS (fully converged)
            has_pred_abs = bool(msg.flags & 0x200)   # EKF_PRED_POS_HORIZ_ABS (good enough for disarmed)
            const_pos    = bool(msg.flags & 0x080)   # EKF_CONST_POS_MODE (bad)
            if gps_ok and has_vel and (has_pos_abs or has_pred_abs) and not const_pos and not ekf_pos_ok:
                ekf_pos_ok = True
                pos_type = "HORIZ_POS_ABS" if has_pos_abs else "PRED_HORIZ_POS_ABS"
                print(f"  EKF position valid ({pos_type}): flags=0x{msg.flags:03x} "
                      f"vel_var={msg.velocity_variance:.2f} "
                      f"pos_var={msg.pos_horiz_variance:.2f}")
        elif mt == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")
        if gps_ok and ekf_pos_ok:
            break
    if not gps_ok:
        print("  WARNING: GPS fix not confirmed, proceeding anyway")
    elif not ekf_pos_ok:
        print("  WARNING: EKF position not validated, proceeding anyway")

    # ── 2. Disable pre-arm checks and switch to GUIDED (while disarmed) ───
    # IMPORTANT ORDER: GUIDED switch must happen while DISARMED.
    # When disarmed, position_ok() uses lenient PRED_HORIZ_POS_ABS check.
    # After arming, position_ok() requires full HORIZ_POS_ABS (stricter).
    print("\n[2] Disabling pre-arm checks (ARMING_CHECK=0)...")
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK', 0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Wait for PARAM_VALUE ack to confirm the param was applied
    ack_deadline = time.time() + 3.0
    while time.time() < ack_deadline:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        if msg.get_type() == 'PARAM_VALUE' and msg.param_id.strip('\x00') == 'ARMING_CHECK':
            print(f"  ARMING_CHECK confirmed = {msg.param_value}")
            break
        if msg.get_type() == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")

    print("\n[3] Setting GUIDED mode (while disarmed — uses lenient EKF check) ...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['GUIDED'])
    if not wait_for_mode(master, master.mode_mapping()['GUIDED'], timeout=5):
        print("  WARNING: GUIDED mode not confirmed, proceeding anyway...")
    else:
        print("  GUIDED mode confirmed")

    # ── 4. Arm in GUIDED ──────────────────────────────────────────────────
    print("\n[4] Arming (force, in GUIDED) ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 2989, 0, 0, 0, 0, 0)   # 2989 = magic_force_arm_value

    armed = False
    arm_deadline = time.time() + 8.0
    while time.time() < arm_deadline:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")
        elif t == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"  [ACK] arm result={msg.result}")
            if msg.result != 0:
                drain(master, 1.0)
                print("  ERROR: arm rejected by FC"); return 1
        elif t == 'HEARTBEAT':
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True; break
    if not armed:
        print("  ERROR: arm timeout"); return 1
    print("  Armed!")

    print("\n[5] Taking off to 45m (GUIDED) ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 45.0)    # 45 m

    # Wait for NAV_TAKEOFF ACK
    to_ack_deadline = time.time() + 3.0
    while time.time() < to_ack_deadline:
        msg = master.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print(f"  NAV_TAKEOFF ACK result={msg.result}")
            break
        elif mt == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")

    last_alt_print = 0.0
    t_takeoff = time.time()
    while time.time() - t_takeoff < 60:
        loop_t = time.time()
        p = get_position(master)
        # Drain messages (non-blocking)
        for _ in range(5):
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'STATUSTEXT':
                print(f"  [FC] {msg.text.strip()}")
        elapsed = time.time() - t_takeoff
        if elapsed - last_alt_print >= 2.0:
            last_alt_print = elapsed
            alt_str = f"{-p[2]:.2f}m" if p else "?"
            print(f"  [takeoff {elapsed:.0f}s] alt={alt_str}")
        if p and -p[2] >= 43.0:
            print(f"  Altitude {-p[2]:.2f}m — takeoff complete")
            fallback_pos = p   # save last confirmed position as emergency fallback
            break
        time.sleep(max(0.0, 0.5 - (time.time() - loop_t)))

    # ── 5. Switch to Mode 99 ──────────────────────────────────────────────
    print("\n[6] Switching to Mode 99 (SMART_PHOTO) ...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        99)

    # Single unified loop: confirm mode 99 AND capture M99_REF_* simultaneously.
    # Do NOT use drain() here — it would discard the NAMED_VALUE_FLOAT messages.
    # Mode 99 broadcasts M99_REF_N/E/D at 1Hz until first companion command;
    # COMPANION_TIMEOUT_MS is 12s, so we have plenty of time to receive them.
    mode99_confirmed = False
    ref_n, ref_e, ref_d = None, None, None
    deadline = time.time() + 10.0
    print("  Waiting for Mode 99 confirmation + initial reference (M99_REF_*)...")
    while time.time() < deadline:
        if ref_n is not None and ref_e is not None and ref_d is not None and mode99_confirmed:
            break
        msg = master.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == 'HEARTBEAT' and msg.get_srcSystem() == master.target_system:
            if msg.custom_mode == 99:
                if not mode99_confirmed:
                    print("  Mode 99 active!")
                    mode99_confirmed = True
        elif mt == 'NAMED_VALUE_FLOAT':
            name = msg.name.rstrip('\x00')
            if name == 'M99_REF_N':
                ref_n = msg.value
            elif name == 'M99_REF_E':
                ref_e = msg.value
            elif name == 'M99_REF_D':
                ref_d = msg.value
        elif mt == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")

    if not mode99_confirmed:
        print("  WARNING: Mode 99 not confirmed")
    if None in (ref_n, ref_e, ref_d):
        print("  WARNING: M99_REF not received, falling back to last takeoff position")
        fb = fallback_pos if 'fallback_pos' in dir() else None
        ref_n = fb[0] if fb else 0.0
        ref_e = fb[1] if fb else 0.0
        ref_d = fb[2] if fb else -45.0

    print(f"  Hold reference: N={ref_n:.2f} E={ref_e:.2f} D={ref_d:.2f} (alt={-ref_d:.2f}m)")

    # ── 6. Hover for 15s at 20Hz ──────────────────────────────────────────
    print("\n[7] Hovering at 20Hz for 15 seconds ...")
    run_at_20hz(master, duration_s=15.0,
                pos_n=ref_n, pos_e=ref_e, pos_d=ref_d,
                vel_n=0.0, vel_e=0.0, vel_d=0.0,
                label="HOVER")

    # ── 7. Climb 3m while holding North/East ─────────────────────────────
    # Ramp pos_d from ref_d to climb_d at 0.5 m/s (6s ramp + 4s hold at top)
    # pos_d_start=ref_d prevents step-change that would saturate thrust controller
    print("\n[8] Climbing 3m at 0.5 m/s (ramped pos_d) ...")
    climb_d = ref_d - 3.0   # 3m up in NED = subtract 3
    run_at_20hz(master, duration_s=10.0,
                pos_n=ref_n, pos_e=ref_e, pos_d=climb_d,
                vel_n=0.0, vel_e=0.0, vel_d=-0.5,   # NED: negative = up
                pos_d_start=ref_d,                   # ramp from hover alt, not jump
                label="CLIMB")

    # ── 8. Hold at new altitude ───────────────────────────────────────────
    print("\n[9] Holding new altitude for 10s ...")
    run_at_20hz(master, duration_s=10.0,
                pos_n=ref_n, pos_e=ref_e, pos_d=climb_d,
                vel_n=0.0, vel_e=0.0, vel_d=0.0,
                label="HOLD")

    # ── 9. Switch to LAND ────────────────────────────────────────────────
    print("\n[10] Switching to LAND mode ...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping()['LAND'])
    drain(master, 5.0)

    print("\n" + "=" * 70)
    print("Companion script complete. SITL is landing.")
    print("=" * 70)
    master.close()
    return 0

if __name__ == '__main__':
    sys.exit(main())
