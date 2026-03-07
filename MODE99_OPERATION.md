# Mode 99 (SMART_PHOTO) — Operation Manual

## Overview

Mode 99 is a custom ArduCopter flight mode implementing **LQI (Linear Quadratic Integral)
state feedback control**. All mission logic runs on a companion computer (PC/Raspberry Pi);
ArduPilot handles only low-level stabilization and safety.

```
Companion PC (20 Hz)              ArduCopter Mode 99 (100 Hz)
┌──────────────────────┐          ┌──────────────────────────────┐
│ State machine        │          │ get_ekf_states()             │
│ Trajectory planning  │─────────>│ update_companion_command()   │
│ SET_POSITION_TARGET  │ MAVLink  │ check_failsafes()            │
│                      │  TCP     │ compute_lqi_control()        │
│ Receives telemetry   │<─────────│   u = u_hover - K*(x-x_ref) │
│ (wind, EKF status)   │          │ mix_motors_from_lqr()        │
└──────────────────────┘          └──────────────────────────────┘
```

**Coordinate system:** NED (North-East-Down), SI units throughout.
- `pos_d` negative = above origin (e.g. 5 m AGL → `pos_d = -5.0`)
- `vel_d` negative = climbing

---

## Files

| File | Location | Purpose |
|---|---|---|
| `run_mode99.sh` | `~/autonomous_drone_sim/` | **Master operation script** |
| `lqr_design.py` | `~/autonomous_drone_sim/` | Computes K gains from sysid parameters |
| `companion_mode99.py` | `~/autonomous_drone_sim/` | Companion state machine (20 Hz) |
| `sysid_params.txt` | `/tmp/sitl_cowork/` | Physical parameters of the drone |
| `lqr_gains.txt` | `/tmp/sitl_cowork/` | K matrix (4×18), auto-generated |
| `arducopter` (binary) | `~/ardupilot/build/sitl/bin/` | ArduCopter SITL firmware |
| `mode_smartphoto99.cpp` | `~/ardupilot/ArduCopter/` | Mode 99 firmware implementation |

---

## Quick Start

```bash
cd ~/autonomous_drone_sim

# Full operation: gains → SITL → companion
./run_mode99.sh
```

That is all. The script handles everything automatically.

---

## Step-by-Step Explanation

### Step 1 — Generate K gains (`lqr_design.py`)

The script reads `sysid_params.txt` and constructs the **linearized quadrotor state space model**:

```
State (12):  [pos_N, pos_E, pos_D, vel_N, vel_E, vel_D, φ, θ, ψ, p, q, r]
Input (4):   [ΔF_thrust, M_roll, M_pitch, M_yaw]

A matrix key entries:
  dvN/dt = g·θ   (pitch → North acceleration)
  dvE/dt = g·φ   (roll  → East  acceleration)
  dvD/dt = -ΔF/m (thrust → vertical acceleration)

B matrix:
  dp/dt  = M_roll  / Ixx
  dq/dt  = M_pitch / Iyy
  dr/dt  = M_yaw   / Izz
```

Six integral states are added (LQI) and the **discrete-time LQR (DARE)** is solved at dt=0.01 s.

**Output K[1][1] (East position → roll moment) comparison:**

| Method | K[1][1] | Motor saturation at |
|---|---|---|
| Old heuristic | 2.94 Nm/m | 0.95 m |
| DARE (physics-based) | **1.01 Nm/m** | **2.79 m** ✓ |

The generated `lqr_gains.txt` is loaded by Mode 99 firmware on every `init()`.

### Step 2 — Start SITL

SITL is started as a background process with:
- `--model +` (plus/X quadrotor configuration)
- `--home 35.3629,138.7274,0,0` (Fuji area, adjust as needed)
- `eeprom.bin` deleted first to ensure clean parameter state

The script waits up to 30 s for port 5760 to open before proceeding.

### Step 3 — Run companion (`companion_mode99.py`)

The companion executes this state machine:

```
CONNECT → GPS_WAIT → GUIDED → ARM → TAKEOFF → MODE99 → HOVER → CLIMB → HOLD → LAND
```

| Phase | Duration | What happens |
|---|---|---|
| CONNECT | until ready | TCP connect to 127.0.0.1:5760, wait heartbeat |
| GPS_WAIT | until EKF ok | Wait for GPS fix and EKF convergence |
| GUIDED | instant | Switch to GUIDED, arm motors |
| TAKEOFF | until 4.5 m | ArduPilot built-in takeoff to ~5 m |
| MODE99 | instant | Switch to Mode 99, receive M99_REF_* |
| HOVER | 15 s | Hold position at takeoff altitude |
| CLIMB | until +3 m | Ramp pos_d at 0.5 m/s to 3 m above hover |
| HOLD | 10 s | Hold new altitude |
| LAND | until ground | Switch to LAND mode |

---

## sysid_params.txt Reference

Location: `~/ardupilot/ArduCopter/sysid_params.txt`

This file is the **single source of truth** for all physical parameters. All models
(PyBullet URDF, ArduPilot Mode 99, RL gym environment) are synchronized to these values.

```ini
MASS=2.0            # Vehicle mass [kg]
IXX=0.0347          # Roll moment of inertia [kg·m²]
IYY=0.0458          # Pitch moment of inertia [kg·m²]
IZZ=0.0977          # Yaw moment of inertia [kg·m²]
MOTOR_KV=920.0      # Motor KV rating
MAX_THRUST=8.0      # Max thrust per motor [N]
ARM_LENGTH=0.225    # Motor-to-center arm length [m]
MOMENT_COEFF=0.016  # Drag torque / thrust ratio  (km = 0.016 × kf = 1.6e-7)
THROTTLE_HOVER=0.5  # Hover throttle fraction [0-1]
```

After changing any parameter:
1. Run `./run_mode99.sh --gains-only` to regenerate K gains
2. Rebuild ArduCopter: `cd ~/ardupilot && ./waf copter`

---

## Gain Tuning (lqr_design.py)

The `Q_DIAG` and `R_DIAG` arrays at the top of `lqr_design.py` control the trade-off
between tracking accuracy and control effort.

```python
Q_DIAG = np.array([
    1.0,  1.0,  2.0,    # pos N, E, D       — increase for tighter position hold
    1.0,  1.0,  2.0,    # vel N, E, D       — increase for faster velocity response
    100., 100., 20.,    # roll, pitch, yaw  — attitude stiffness
    4.0,  4.0,  1.0,    # p, q, r rates
    0.25, 0.25, 0.5,    # integral pos N,E,D — increase slowly (anti-windup risk)
    1.0,  1.0,  2.0,    # integral vel N,E,D
])
R_DIAG = np.array([0.04, 0.16, 0.16, 6.25])  # thrust, M_roll, M_pitch, M_yaw
```

**Rules:**
- **Larger Q / smaller R** → more aggressive (tighter tracking, more moment commands)
- **Smaller Q / larger R** → softer (less saturation, slower response)
- **Do NOT increase R[1] or R[2] above ~0.5** — the DARE solver fails when R is too
  large relative to the system's control authority (all A eigenvalues are at 0)
- After any Q/R change, verify the **physical sanity check** output:
  ```
  East pos error at motor saturation: 2.79 m   ← must be > 1.0 m
  GOOD (saturation above 1.0 m)
  ```

---

## Expected Terminal Output

```
[1/4] Generating LQR K gains...
  K[thr][pD]=-12.38  K[rol][pE]=1.01  K[rol][attR]=6.77
  East pos error at motor saturation: 2.79 m — GOOD

[2/4] Cleaning up old SITL...
[3/4] Starting ArduCopter SITL ... SITL ready on port 5760

[4/4] Running companion script...
[CONNECT ] Connected. Got heartbeat sysid=1
[GPS_WAIT] Waiting for EKF...
[GPS_WAIT] GPS OK. EKF healthy.
[GUIDED  ] Mode switch: GUIDED
[ARM     ] Armed
[TAKEOFF ] Climbing to 5.0 m...
[MODE99  ] Entered Mode 99
[MODE99  ] Got M99_REF_D = -4.74 m
[HOVER   ] Holding for 15 s  pos_d=-4.74 m
[CLIMB   ] Ramping to -7.74 m at 0.5 m/s
[HOLD    ] Holding for 10 s  pos_d=-7.74 m
[LAND    ] Landing...
```

In the SITL log (`/tmp/sitl_cowork/sitl_run.txt`) you should see:

```
SMARTPHOTO99: K loaded from lqr_gains.txt (4x18 LQI)
  K[thr][pD]=-12.383 K[rol][pE]=1.006 K[rol][attR]=6.772
MODE99: LQI State Feedback @ 100Hz
M99 ref=-4.74 cur=-4.76 thr=0.389 int=0.02
```

---

## Failsafes

Mode 99 automatically transitions to **LAND** mode on any of:

| Condition | Threshold | Check rate |
|---|---|---|
| Companion timeout | No command for 12 s | 400 Hz |
| Battery critical | < 20% remaining | 400 Hz |
| EKF unhealthy | `ahrs.healthy()` false | 400 Hz |
| GPS degraded | < 10 sats or HDOP > 1.5 | 400 Hz |

The 12 s timeout covers: drain (1 s) + wait_for_mode (5 s) + M99_REF wait (5 s).

---

## Troubleshooting

### SITL does not start / port 5760 never opens

```bash
tail -20 /tmp/sitl_cowork/sitl_run.txt
```

Common causes:
- Old `eeprom.bin` with corrupted parameters → script removes it automatically
- Another process on port 5760 → `pkill -9 -f arducopter`

### Companion immediately enters LAND (companion timeout)

SITL takes longer than expected to initialize EKF. The 12 s timeout starts at
Mode 99 entry. If EKF is slow, increase `COMPANION_TIMEOUT_MS` in
`mode_smartphoto99.h` and rebuild.

### M_roll runaway / altitude climb to 100 m

Verify `lqr_gains.txt` is being loaded (look for `K loaded from lqr_gains.txt`
in SITL log). If the message says `lqr_gains.txt not found`, regenerate:

```bash
./run_mode99.sh --gains-only
```

### DARE solver fails in lqr_design.py

```
Failed to find a finite solution
```

R values are too large. Keep R[1] ≤ 0.5. Reduce Q instead of increasing R to
soften the controller.

### Build the firmware after code changes

```bash
cd ~/ardupilot
./waf copter      # ~22 s incremental build
```

---

## Architecture Reference

### Control Law (100 Hz)

```
Error state e[18]:
  e[0..2]   = pos_N/E/D - ref_N/E/D          [m]
  e[3..5]   = vel_N/E/D - ref_N/E/D          [m/s]
  e[6..8]   = att_err from quaternion        [rad]
  e[9..11]  = p,q,r - ref_p,q,r             [rad/s]
  e[12..14] = ∫(pos - ref) dt               [m·s]
  e[15..17] = ∫(vel - ref) dt               [(m/s)·s]

Control output:
  u[0] = hover_thrust_N  - K[0]·e   [N]    → throttle
  u[1] = 0              - K[1]·e   [Nm]   → roll moment
  u[2] = 0              - K[2]·e   [Nm]   → pitch moment
  u[3] = 0              - K[3]·e   [Nm]   → yaw moment
```

### Motor Mixing (X-config, FL=0 FR=1 RL=2 RR=3)

```
motor[0] = F/4 - M_roll/(4L) - M_pitch/(4L) + M_yaw/(4kM)   FL
motor[1] = F/4 + M_roll/(4L) - M_pitch/(4L) - M_yaw/(4kM)   FR
motor[2] = F/4 - M_roll/(4L) + M_pitch/(4L) - M_yaw/(4kM)   RL
motor[3] = F/4 + M_roll/(4L) + M_pitch/(4L) + M_yaw/(4kM)   RR
```

Physical max roll moment at hover: `F_hover/4 × 4 × L = 3.11 × 0.90 = 2.80 Nm`

### MAVLink Interface

| Direction | Message | Rate | Content |
|---|---|---|---|
| Companion → FC | `SET_POSITION_TARGET_LOCAL_NED` | 20 Hz | pos NED [m], vel NED [m/s], yaw [rad], yaw_rate [rad/s] |
| FC → Companion | `NAMED_VALUE_FLOAT` | 1 Hz | LQI_Thrust, M_roll/pitch/yaw, MTR0-3, EKF states, integrals |
| FC → Companion | `NAMED_VALUE_FLOAT` | 1 Hz | WindSpd, WindDir, WindN/E/D |
| FC → Companion | `NAMED_VALUE_FLOAT` | 1 Hz (on init) | M99_REF_N/E/D |
