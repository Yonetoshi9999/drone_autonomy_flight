# Customizing SITL Physical Parameters with a Frame Model JSON

## Background: Why Physical Parameters Must Match Your Controller

ArduPilot SITL uses the following default physical parameters out of the box:

```
mass              = 3.0 kg  (hardcoded default in SIM_Frame.h line 85)
moment_of_inertia = estimated from mass × diagonal_size²  (geometric approximation)
```

For standard ArduPilot PID controllers this rarely matters — gains are tuned
empirically (e.g. via AutoTune), so the controller adapts to whatever the SITL
dynamics happen to be.

**For model-based controllers (LQR, MPC, H-infinity, etc.) it is critical.**
The gain matrix is computed analytically from the physical model, so any mismatch
between the model used for design and the model used in simulation produces
incorrect gains.

### Concrete example: 50 % mass mismatch

```
LQR designed for:  mass = 2.0 kg  →  braking moment = M_brake
SITL running with: mass = 3.0 kg  →  effective deceleration = M_brake / 3.0
                                       (33 % weaker than designed)

Result:
  vel_ref = 1.5 m/s  but  actual speed reaches 3–4 m/s
  → LQR requests maximum pitch moment to brake
  → pitch angle > 40° → TILT / FLIP
```

The same logic applies to moment-of-inertia mismatches: if SITL estimates inertia
from geometry while the real vehicle has a measured value, the angular acceleration
response will differ from the design assumption.

---

## Solution: Custom Frame Model JSON

ArduPilot SITL supports loading a custom physical model via:

```
--model "+:/path/to/frame.json"
```

The JSON file overrides any subset of the default frame parameters.
Only the fields you specify are changed; everything else uses the built-in defaults.

### Minimal JSON to fix mass and inertia

```json
{
    "mass"           : 2.0,
    "diagonal_size"  : 0.45,
    "moment_inertia" : [0.0347, 0.0458, 0.0977],
    "hoverThrOut"    : 0.5,
    "num_motors"     : 4
}
```

| Field | Type | Description | Example |
|---|---|---|---|
| `mass` | float | Total vehicle mass (kg) | `2.0` |
| `diagonal_size` | float | Motor-to-motor diagonal distance (m) = 2 × arm_length | `0.45` |
| `moment_inertia` | [x,y,z] | Principal moments of inertia [Ixx, Iyy, Izz] (kg·m²) | `[0.0347, 0.0458, 0.0977]` |
| `hoverThrOut` | float | Hover throttle fraction (0–1) | `0.5` |
| `num_motors` | int | Number of motors | `4` |

> **Note:** If `moment_inertia` is absent or zero, SITL estimates it from
> `mass × (diagonal_size / 2)²`. Specifying it explicitly eliminates this
> approximation entirely.

### Full list of available fields (from `SIM_Frame.cpp`)

```json
{
    "mass"          : 2.0,        // kg — total vehicle mass
    "diagonal_size" : 0.45,       // m  — motor-to-motor diagonal
    "moment_inertia": [Ixx, Iyy, Izz],  // kg·m² — principal moments
    "hoverThrOut"   : 0.5,        // 0–1 — hover throttle
    "refSpd"        : 10.0,       // m/s — reference airspeed for drag estimation
    "refAngle"      : 15.0,       // deg — pitch angle at refSpd
    "refVoltage"    : 22.2,       // V
    "refCurrent"    : 10.0,       // A
    "refAlt"        : 0,          // m AMSL
    "refTempC"      : 25,         // °C
    "refBatRes"     : 0.02,       // Ω — battery internal resistance
    "maxVoltage"    : 25.2,       // V — full-charge voltage
    "battCapacityAh": 0,          // Ah — 0 means unlimited
    "propExpo"      : 0.65,       // MOT_THST_EXPO equivalent
    "refRotRate"    : 300,        // deg/s — maximum yaw rate
    "pwmMin"        : 1000,       // µs — MOT_PWM_MIN
    "pwmMax"        : 2000,       // µs — MOT_PWM_MAX
    "spin_min"      : 0.10,       // MOT_SPIN_MIN
    "spin_max"      : 0.95,       // MOT_SPIN_MAX
    "slew_max"      : 0,          // max motor slew rate (0 = disabled)
    "disc_area"     : 0.203,      // m² — total effective propeller disc area
    "mdrag_coef"    : 0.10,       // momentum drag coefficient
    "num_motors"    : 4
}
```

---

## How to Apply

### Passing the JSON to the arducopter binary

```bash
/path/to/build/sitl/bin/arducopter \
    --model "+:/absolute/path/to/my_frame.json" \
    --defaults Tools/autotest/default_params/copter.parm,my_params.parm \
    --speedup 5 \
    -I0
```

The `+` selects the standard X-frame quad type; everything after the colon is
the path to the JSON file that overrides its parameters.

### Using a relative path

```bash
# The path is relative to the working directory of the arducopter process.
--model "+:configs/ardupilot/my_frame.json"
```

### Existing examples in the ArduPilot repository

`Tools/autotest/models/` already contains two JSON frame files as reference:

| File | Vehicle | mass |
|---|---|---|
| `freestyle.json` | 5-inch FPV racing quad | 0.8 kg |
| `Callisto.json` | Large octocopter | 32.5 kg |

---

## Verifying It Worked

When SITL loads a JSON file successfully it prints to stdout (also captured in
`/tmp/sitl_mode99.log` or equivalent):

```
Loaded model params from /path/to/my_frame.json
Suggested EK3_DRAG_BCOEF_* = XX.XXX, EK3_DRAG_MCOEF = X.XXX
```

Check with:

```bash
grep "Loaded model\|Suggested EK3" /tmp/sitl_mode99.log
```

If the file path is wrong, ArduPilot will panic at startup:

```
PANIC: /path/to/my_frame.json failed to load
```

---

## Updating Parameters After Re-identification

When system identification is re-run and `sysid_params.txt` (or equivalent) is
updated, keep the three sources in sync:

| File | Parameters to update |
|---|---|
| `configs/ardupilot/my_frame.json` | `mass`, `moment_inertia`, `hoverThrOut` |
| `design_lqr_discrete.py` (or equivalent) | `mass`, `Ixx`, `Iyy`, `Izz` |
| `lqr_gains.txt` | regenerate by re-running the LQR design script |

```bash
# Regenerate LQR gains after updating mass/inertia
python3 design_lqr_discrete.py

# Copy to the path Mode 99 loads at runtime
cp lqr_gains.txt /tmp/sitl_cowork/lqr_gains.txt
```

---

## Technical Notes

### Where SITL applies these parameters (ArduPilot source)

| Location | What it does |
|---|---|
| `libraries/SITL/SIM_Frame.h:85` | Default `mass = 3.0` |
| `libraries/SITL/SIM_Frame.cpp:452` | `Frame::load_frame_params()` — parses JSON |
| `libraries/SITL/SIM_Frame.cpp:630` | Inertia estimation fallback (used when `moment_inertia` is zero) |
| `libraries/SITL/SIM_Frame.cpp:688` | `rot_accel = torque / moment_of_inertia` |
| `libraries/SITL/SIM_Frame.cpp:720` | `body_accel = thrust / gross_mass()` |

### Computing `diagonal_size` from arm length

For a symmetric X-frame where `arm_length` is the center-to-motor distance:

```
diagonal_size = 2 × arm_length
```

Example: `arm_length = 0.225 m` → `diagonal_size = 0.45 m`

When `moment_inertia` is specified directly, `diagonal_size` only affects the
aerodynamic drag model (used at higher airspeeds) and has negligible impact on
hover and low-speed flight dynamics.

### Computing `disc_area` from propeller diameter

```
disc_area = num_motors × π × (diameter / 2)²

Example: 4 × 10-inch (0.254 m) props
  disc_area = 4 × π × 0.127² ≈ 0.203 m²
```

### Impact on model-based controllers

For a controller with gain matrix **K** computed from linearised dynamics:

```
ẍ = F / m          →  m appears in the B matrix (translational)
α = M / I          →  I appears in the B matrix (rotational)

If m_SITL ≠ m_model:
  actual acceleration = K·e / m_SITL  ≠  designed acceleration = K·e / m_model
  → velocity tracking error accumulates → excessive control effort → instability
```

Matching SITL parameters to the design model is therefore **a prerequisite for
validating any model-based flight controller in SITL before deploying to hardware**.
