#!/usr/bin/env python3
"""
Discrete-time LQR design for Mode 99 (18-state LQI controller)

System: quadcopter near hover, NED frame
State error: [pos_N, pos_E, pos_D,  vel_N, vel_E, vel_D,
              att_roll, att_pitch, att_yaw,  p, q, r,
              int_pos_N, int_pos_E, int_pos_D,
              int_vel_N, int_vel_E, int_vel_D]
Control: [F_thrust (delta from hover), M_roll, M_pitch, M_yaw]

Continuous-time linearization near hover (NED, small angles):
  v̇_N = -g * theta       (positive pitch = nose UP → moves SOUTH)
  v̇_E =  g * phi         (positive roll  = right-down → moves EAST)
  v̇_D = -delta_F / m     (more thrust → less pos_D, i.e., goes up)
  θ̈_roll  = M_roll  / Ixx
  θ̈_pitch = M_pitch / Iyy
  θ̈_yaw   = M_yaw   / Izz

Discretized via Zero-Order Hold: A_d = expm(A*dt), B_d = ∫₀ᵈᵗ expm(Aτ)B dτ
Discrete LQR: solve discrete Algebraic Riccati Equation (DARE)
  P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q
  K = (R + B_d^T P B_d)^{-1} B_d^T P A_d

Output: lqr_gains.txt  (4 rows × 18 cols, space-separated)
        Place in ~/ardupilot/ArduCopter/ or /tmp/sitl_cowork/
"""

import json
import numpy as np
from scipy.linalg import expm, solve_discrete_are

# ============================================================================
# System parameters — loaded from quad_2kg.json (single source of truth)
# ============================================================================
_FRAME_MODEL_PATH = (
    f"{__import__('os').path.expanduser('~')}"
    "/ardupilot/Tools/autotest/models/quad_2kg.json"
)
with open(_FRAME_MODEL_PATH) as _f:
    _frame = json.load(_f)

m    = _frame["mass"]                  # kg
Ixx  = _frame["moment_inertia"][0]    # kg·m²
Iyy  = _frame["moment_inertia"][1]    # kg·m²
Izz  = _frame["moment_inertia"][2]    # kg·m²
g    = 9.81     # m/s²
dt   = 0.01     # s  (100 Hz LQR rate)

# ============================================================================
# State indices in the 18-element error vector
# [0..2]=pos, [3..5]=vel, [6..8]=att, [9..11]=rates, [12..14]=int_pos, [15..17]=int_vel
# ============================================================================

# ============================================================================
# Helper: ZOH discretization of (A_c, B_c)
# ============================================================================
def zoh_discretize(Ac, Bc, dt):
    """
    Compute discrete-time (A_d, B_d) via Zero-Order Hold.
    Uses the matrix exponential of the augmented system.
    """
    n = Ac.shape[0]
    m_in = Bc.shape[1]
    # Build augmented matrix [A  B; 0  0]
    M = np.zeros((n + m_in, n + m_in))
    M[:n, :n] = Ac
    M[:n, n:] = Bc
    eM = expm(M * dt)
    Ad = eM[:n, :n]
    Bd = eM[:n, n:]
    return Ad, Bd

# ============================================================================
# Helper: Discrete LQR
# ============================================================================
def dlqr(Ad, Bd, Q, R):
    """
    Solve discrete LQR.
    Returns K such that u = -K @ x minimizes sum(x^T Q x + u^T R u).
    """
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
    return K, P

# ============================================================================
# Q and R weights
# (same philosophy as the heuristic, but now applied to the proper DARE)
# ============================================================================
# --- Q weights for each state ---
# Horizontal position: small (LQR is near hover, large pos error clamped to 2m)
q_pos_ne  = 0.05    # pos_N, pos_E
q_pos_d   = 2.0     # pos_D (altitude — important to hold)
q_vel_ne  = 0.2     # vel_N, vel_E  (0.05→0.5: stronger vel braking to overcome rate damping; 0.5→0.2: reduce tilt)
q_vel_d   = 2.0     # vel_D
q_att_rp  = 0.2     # att roll/pitch (10.0→2.0→0.5→0.2: allow natural tilt during forward flight)
q_att_yaw = 5.0     # att yaw
q_rate_rp = 1.5     # roll/pitch rate (20.0→1.5: allow braking; continuous ζ≈0.8 with q_att=2.0)
q_rate_r  = 0.5     # yaw rate
q_int_pos_ne = 0.0  # DISABLED: random RL cmds → windup → flip
q_int_pos_d  = 0.5  # altitude integral (useful for steady-state correction)
q_int_vel_ne = 0.0  # DISABLED
q_int_vel_d  = 0.1  # altitude velocity integral

# --- R weights for each control input ---
r_F     = 1.0   # F_thrust
r_Mroll = 5.0   # M_roll  (1.0→2.0→5.0: reduce tilt by suppressing moment output)
r_Mpitch= 5.0   # M_pitch (1.0→2.0→5.0: reduce tilt by suppressing moment output)
r_Myaw  = 2.0   # M_yaw

# ============================================================================
# SUBSYSTEM 1: North / Pitch
# States: [pos_N, vel_N, att_pitch, rate_q]  (4-state, maps to error idx [0,3,7,10])
# Input:  M_pitch
# ============================================================================
# Continuous A:
#   d/dt pos_N     = vel_N
#   d/dt vel_N     = -g * att_pitch  (ArduPilot: positive att_pitch = nose UP → moves SOUTH)
#   d/dt att_pitch = rate_q          (ArduPilot: positive q = pitching nose UP)
#   d/dt rate_q    = +M_pitch / Iyy  (Mode99 reverse-mix + AP_Motors: set_pitch(+) → front motors MORE
#                                      → NOSE UP → q INCREASES → B sign is POSITIVE)
Ac_NP = np.array([
    [0, 1,  0,       0],
    [0, 0, -g,       0],
    [0, 0,  0,       1],
    [0, 0,  0,       0],
], dtype=float)
Bc_NP = np.array([[0], [0], [0], [1/Iyy]])
Ad_NP, Bd_NP = zoh_discretize(Ac_NP, Bc_NP, dt)
Q_NP = np.diag([q_pos_ne, q_vel_ne, q_att_rp, q_rate_rp])
R_NP = np.array([[r_Mpitch]])
K_NP, _ = dlqr(Ad_NP, Bd_NP, Q_NP, R_NP)
# K_NP[0] = [K_posN, K_velN, K_attPitch, K_rateQ] → M_pitch row

print("=== North/Pitch subsystem ===")
print(f"  K_NP = {K_NP[0]}")
print(f"  ωn = {np.sqrt(q_att_rp/r_Mpitch * Iyy**0 ):.3f} rad/s  (rough estimate)")

# ============================================================================
# SUBSYSTEM 2: East / Roll
# States: [pos_E, vel_E, att_roll, rate_p]  (maps to error idx [1,4,6,9])
# Input:  M_roll
# ============================================================================
# Continuous A:
#   d/dt pos_E   = vel_E
#   d/dt vel_E   = +g * att_roll  (right-roll → east acceleration)
#   d/dt att_roll = rate_p
#   d/dt rate_p  = M_roll / Ixx
Ac_ER = np.array([
    [0, 1,  0,      0],
    [0, 0,  g,      0],   # +g for east: positive roll → east acceleration
    [0, 0,  0,      1],
    [0, 0,  0,      0],
], dtype=float)
Bc_ER = np.array([[0], [0], [0], [1/Ixx]])
Ad_ER, Bd_ER = zoh_discretize(Ac_ER, Bc_ER, dt)
Q_ER = np.diag([q_pos_ne, q_vel_ne, q_att_rp, q_rate_rp])
R_ER = np.array([[r_Mroll]])
K_ER, _ = dlqr(Ad_ER, Bd_ER, Q_ER, R_ER)
# K_ER[0] = [K_posE, K_velE, K_attRoll, K_rateP] → M_roll row

print("\n=== East/Roll subsystem ===")
print(f"  K_ER = {K_ER[0]}")

# ============================================================================
# SUBSYSTEM 3: Altitude / Thrust
# States: [pos_D, vel_D]  (maps to error idx [2, 5])
# Input:  delta_F = F_total - hover_thrust
# ============================================================================
# d/dt pos_D = vel_D
# d/dt vel_D = -delta_F / m  (more thrust → less pos_D = rising)
Ac_alt = np.array([
    [0, 1],
    [0, 0],
], dtype=float)
Bc_alt = np.array([[0], [-1/m]])
Ad_alt, Bd_alt = zoh_discretize(Ac_alt, Bc_alt, dt)
Q_alt = np.diag([q_pos_d, q_vel_d])
R_alt = np.array([[r_F]])
K_alt, _ = dlqr(Ad_alt, Bd_alt, Q_alt, R_alt)
# K_alt[0] = [K_posD, K_velD] → F_thrust row (delta_F)

print("\n=== Altitude/Thrust subsystem ===")
print(f"  K_alt = {K_alt[0]}")
print(f"  (applied as F_total = hover_thrust - K_alt @ [e_posD, e_velD])")

# ============================================================================
# SUBSYSTEM 4: Yaw
# States: [att_yaw, rate_r]  (maps to error idx [8, 11])
# Input:  M_yaw
# ============================================================================
Ac_yaw = np.array([[0, 1], [0, 0]], dtype=float)
Bc_yaw = np.array([[0], [1/Izz]])
Ad_yaw, Bd_yaw = zoh_discretize(Ac_yaw, Bc_yaw, dt)
Q_yaw = np.diag([q_att_yaw, q_rate_r])
R_yaw = np.array([[r_Myaw]])
K_yaw, _ = dlqr(Ad_yaw, Bd_yaw, Q_yaw, R_yaw)

print("\n=== Yaw subsystem ===")
print(f"  K_yaw = {K_yaw[0]}")

# ============================================================================
# ALTITUDE INTEGRAL: include int_pos_D (idx 14) and int_vel_D (idx 17)
# ============================================================================
# Extended altitude subsystem: [pos_D, vel_D, int_pos_D, int_vel_D]
# d/dt int_pos_D = e_pos_D  (but stored as -e in Mode99, corrected by sign in e[14]=-int_pos_d)
# d/dt int_vel_D = e_vel_D  (similarly e[17] = -int_vel_d)
# For the LQR, we design as if the error state is [pos_D, vel_D, int_pos_D, int_vel_D]
# where int accumulates pos_D and vel_D errors.
# Note: Mode99 stores int = ∫(ref-cur), e=-int = ∫(cur-ref). The K sign accounts for this.
if q_int_pos_d > 0 or q_int_vel_d > 0:
    Ac_alt_i = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [1, 0, 0, 0],   # d/dt int_pos_D = pos_D error
        [0, 1, 0, 0],   # d/dt int_vel_D = vel_D error
    ], dtype=float)
    Bc_alt_i = np.array([[0], [-1/m], [0], [0]])
    Ad_alt_i, Bd_alt_i = zoh_discretize(Ac_alt_i, Bc_alt_i, dt)
    Q_alt_i = np.diag([q_pos_d, q_vel_d, q_int_pos_d, q_int_vel_d])
    R_alt_i = np.array([[r_F]])
    K_alt_i, _ = dlqr(Ad_alt_i, Bd_alt_i, Q_alt_i, R_alt_i)
    print(f"\n=== Altitude with integrals ===")
    print(f"  K_alt_i = {K_alt_i[0]}")
    K_alt_posD = K_alt_i[0, 0]
    K_alt_velD = K_alt_i[0, 1]
    K_alt_intPosD = K_alt_i[0, 2]
    K_alt_intVelD = K_alt_i[0, 3]
else:
    K_alt_posD = K_alt[0, 0]
    K_alt_velD = K_alt[0, 1]
    K_alt_intPosD = 0.0
    K_alt_intVelD = 0.0

# ============================================================================
# Assemble full 4×18 K matrix
# ============================================================================
# Row 0: F_thrust  (delta from hover)
# Row 1: M_roll
# Row 2: M_pitch
# Row 3: M_yaw
#
# State indices:
# [0]=pos_N [1]=pos_E [2]=pos_D [3]=vel_N [4]=vel_E [5]=vel_D
# [6]=att_roll [7]=att_pitch [8]=att_yaw
# [9]=rate_p [10]=rate_q [11]=rate_r
# [12]=int_pos_N [13]=int_pos_E [14]=int_pos_D
# [15]=int_vel_N [16]=int_vel_E [17]=int_vel_D

K_full = np.zeros((4, 18))

# --- F_thrust row (row 0) ---
K_full[0, 2]  = K_alt_posD     # pos_D
K_full[0, 5]  = K_alt_velD     # vel_D
K_full[0, 14] = K_alt_intPosD  # int_pos_D
K_full[0, 17] = K_alt_intVelD  # int_vel_D

# --- M_roll row (row 1) ---
# K_ER: [K_posE, K_velE, K_attRoll, K_rateP]
K_full[1, 1]  = K_ER[0, 0]    # pos_E
K_full[1, 4]  = K_ER[0, 1]    # vel_E
K_full[1, 6]  = K_ER[0, 2]    # att_roll
K_full[1, 9]  = K_ER[0, 3]    # rate_p

# --- M_pitch row (row 2) ---
# K_NP: [K_posN, K_velN, K_attPitch, K_rateQ]
K_full[2, 0]  = K_NP[0, 0]    # pos_N
K_full[2, 3]  = K_NP[0, 1]    # vel_N
K_full[2, 7]  = K_NP[0, 2]    # att_pitch
K_full[2, 10] = K_NP[0, 3]    # rate_q

# --- M_yaw row (row 3) ---
# K_yaw: [K_attYaw, K_rateR]
K_full[3, 8]  = K_yaw[0, 0]   # att_yaw
K_full[3, 11] = K_yaw[0, 1]   # rate_r

# ============================================================================
# Print full matrix
# ============================================================================
print("\n=== Full 4×18 K matrix ===")
labels = ['pos_N','pos_E','pos_D','vel_N','vel_E','vel_D',
          'att_r','att_p','att_y','p','q','r',
          'ipos_N','ipos_E','ipos_D','ivel_N','ivel_E','ivel_D']
print(f"{'':12s}", end="")
for l in labels:
    print(f"{l:>8s}", end="")
print()
row_names = ['F_thrust', 'M_roll', 'M_pitch', 'M_yaw']
for i, rn in enumerate(row_names):
    print(f"{rn:12s}", end="")
    for j in range(18):
        print(f"{K_full[i,j]:8.4f}", end="")
    print()

# ============================================================================
# Stability check: closed-loop eigenvalues for each subsystem
# ============================================================================
print("\n=== Closed-loop eigenvalues ===")
Acl_NP = Ad_NP - Bd_NP @ K_NP
print(f"  North/Pitch: {np.abs(np.linalg.eigvals(Acl_NP))}")
Acl_ER = Ad_ER - Bd_ER @ K_ER
print(f"  East/Roll:   {np.abs(np.linalg.eigvals(Acl_ER))}")
Acl_alt = Ad_alt - Bd_alt @ K_alt[:, :2]
print(f"  Altitude:    {np.abs(np.linalg.eigvals(Acl_alt))}")
Acl_yaw = Ad_yaw - Bd_yaw @ K_yaw
print(f"  Yaw:         {np.abs(np.linalg.eigvals(Acl_yaw))}")

# All eigenvalues should be inside unit circle (|λ| < 1) for stability

# ============================================================================
# Write lqr_gains.txt
# ============================================================================
output_path = "/home/yonetoshi27/ardupilot/ArduCopter/lqr_gains.txt"
with open(output_path, "w") as f:
    f.write("# Discrete LQR gains (4x18) for Mode 99\n")
    f.write("# Generated by design_lqr_discrete.py\n")
    f.write(f"# dt={dt}s (100Hz), ZOH discretization, DARE solution\n")
    f.write(f"# m={m}kg, Ixx={Ixx}, Iyy={Iyy}, Izz={Izz}\n")
    f.write(f"# Q: pos_ne={q_pos_ne} pos_d={q_pos_d} vel_ne={q_vel_ne} vel_d={q_vel_d}\n")
    f.write(f"#    att_rp={q_att_rp} att_y={q_att_yaw} rate_rp={q_rate_rp} rate_r={q_rate_r}\n")
    f.write(f"#    int_pos_ne={q_int_pos_ne} int_pos_d={q_int_pos_d}\n")
    f.write(f"#    int_vel_ne={q_int_vel_ne} int_vel_d={q_int_vel_d}\n")
    f.write(f"# R: F={r_F} Mroll={r_Mroll} Mpitch={r_Mpitch} Myaw={r_Myaw}\n")
    f.write(f"# Rows: [F_thrust, M_roll, M_pitch, M_yaw]\n")
    f.write(f"# Cols: [pos_N,pos_E,pos_D,vel_N,vel_E,vel_D,att_r,att_p,att_y,"
            f"p,q,r,ipos_N,ipos_E,ipos_D,ivel_N,ivel_E,ivel_D]\n")
    for i in range(4):
        row_str = " ".join(f"{K_full[i, j]:.8f}" for j in range(18))
        f.write(row_str + "\n")

print(f"\n✅ Written to {output_path}")

# Also copy to sitl_cowork for runtime loading
import os, shutil
sitl_dir = "/tmp/sitl_cowork"
os.makedirs(sitl_dir, exist_ok=True)
shutil.copy(output_path, f"{sitl_dir}/lqr_gains.txt")
print(f"✅ Copied to {sitl_dir}/lqr_gains.txt")
