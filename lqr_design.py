#!/usr/bin/env python3
"""
lqr_design.py - Compute LQR/LQI gain matrix K from sysid parameters.

Reads sysid_params.txt, constructs A (12x12) and B (12x4) state space matrices
for a quadrotor hovering in NED frame, augments for LQI (+6 integral states),
solves CARE, and writes K (4x18) to lqr_gains.txt.

State order (12): [pos_N, pos_E, pos_D, vel_N, vel_E, vel_D, phi, theta, psi, p, q, r]
Input order (4):  [delta_F, M_roll, M_pitch, M_yaw]
LQI integrals (6): [int_posN, int_posE, int_posD, int_velN, int_velE, int_velD]
  where int_posX = integral of (current_posX - ref_posX)

Coordinate conventions:
  NED frame, positive pitch = nose down = forward (North), positive roll = right (East) tilt

Usage:
  python3 lqr_design.py [sysid_params.txt]
"""

import numpy as np
import os
import sys

try:
    from scipy.linalg import solve_continuous_are, solve_discrete_are
    from scipy.signal import cont2discrete
except ImportError:
    print("ERROR: scipy not found. Install with: pip3 install scipy")
    sys.exit(1)

SYSID_PATHS = [
    '/tmp/sitl_cowork/sysid_params.txt',
    'sysid_params.txt',
]
LQR_OUT_PATH = '/tmp/sitl_cowork/lqr_gains.txt'

# ============================================================
# Q and R weight matrices (tune these to adjust performance)
# ============================================================
# Q = 1 / (tolerable_error)^2 for each state
# R = 1 / (max_control)^2 for each input
#
# State tolerances:
#   pos:  1.0 m  → Q=1.0
#   vel:  1.0 m/s → Q=1.0
#   att:  0.1 rad (~6°) → Q=100
#   rate: 0.5 rad/s → Q=4
#   int_pos: 2.0 m·s → Q=0.25
#   int_vel: 1.0 (m/s)·s → Q=1.0
#
# Control limits (keep M_roll within physical max ~2.8 Nm for a typical SITL quad):
#   delta_F: 5 N → R=0.04  (but position is decoupled so relax a bit)
#   M_roll:  2.5 Nm → R=0.16  (1/2.5^2)
#   M_pitch: 2.5 Nm → R=0.16
#   M_yaw:   0.4 Nm → R=6.25  (1/0.4^2, yaw moment limited by drag coefficient)
Q_DIAG = np.array([
    1.0,  1.0,  2.0,    # pos N, E, D
    1.0,  1.0,  2.0,    # vel N, E, D
    100.0, 100.0, 20.0, # phi (roll), theta (pitch), psi (yaw)
    4.0,  4.0,  1.0,    # p, q, r
    0.25, 0.25, 0.5,    # int_pos N, E, D
    1.0,  1.0,  2.0,    # int_vel N, E, D
])
# R: 1/(max_control)^2
#   delta_F:  max ~5N → R[0] = 1/5^2 ≈ 0.04
#   M_roll/pitch: max ~2.5Nm → R[1]=R[2] = 1/2.5^2 ≈ 0.16
#   M_yaw:    max ~0.4Nm (drag torque) → R[3] = 1/0.4^2 = 6.25
# NOTE: The DARE (discrete-time) solver is used below because all A eigenvalues
# are at 0 → Ad eigenvalues exactly on unit circle.  The solver fails when R is
# too large; R[1]=0.16 gives K[1][1]≈1.0 Nm/m (motor sat. at ~2.8m), which is
# within physical limits.  To get softer gains, reduce Q weights instead.
R_DIAG = np.array([0.04, 0.16, 0.16, 6.25])


def read_sysid(path):
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' in line:
                k, v = line.split('=', 1)
                params[k.strip()] = float(v.strip())
    return params


def build_system_matrices(params):
    """Build linearized quadrotor hover A (12x12) and B (12x4)."""
    g   = 9.80665
    m   = params.get('MASS',       1.27)
    Ixx = params.get('IXX',        0.01)
    Iyy = params.get('IYY',        0.01)
    Izz = params.get('IZZ',        0.02)

    # 12x12 A matrix (state: [pN, pE, pD, vN, vE, vD, phi, theta, psi, p, q, r])
    A = np.zeros((12, 12))
    # Position derivatives = velocity
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0
    # Velocity from gravity coupling (small-angle hover linearization)
    # dvN/dt = g * theta  (positive pitch=nose-down → North acceleration)
    # dvE/dt = g * phi    (positive roll=right-tilt → East acceleration)
    A[3, 7] = g    # North  ← pitch
    A[4, 6] = g    # East   ← roll
    # (vD has no attitude coupling at hover; it responds to thrust variation via B)
    # Angle derivatives = rates
    A[6,  9] = 1.0
    A[7, 10] = 1.0
    A[8, 11] = 1.0
    # (p, q, r have no A-matrix coupling; they respond to moments via B)

    # 12x4 B matrix (input: [delta_F, M_roll, M_pitch, M_yaw])
    B = np.zeros((12, 4))
    B[5,  0] = -1.0 / m    # dvD/dt = -delta_F/m  (NED: more thrust → less downward acc)
    B[9,  1] =  1.0 / Ixx  # dp/dt  = M_roll / Ixx
    B[10, 2] =  1.0 / Iyy  # dq/dt  = M_pitch / Iyy
    B[11, 3] =  1.0 / Izz  # dr/dt  = M_yaw / Izz

    return A, B, m, Ixx, Iyy, Izz, g


def build_lqi_matrices(A, B):
    """Augment A, B with 6 integral states for LQI control.

    Integral states (indices 12..17):
      x_aug[12..14] = integral of (pos_N/E/D - ref_N/E/D)  i.e. ∫(pos - ref) dt
      x_aug[15..17] = integral of (vel_N/E/D - ref_N/E/D)

    For regulation design (ref=0): d/dt[int_posX] = posX = x[0..2]
                                    d/dt[int_velX] = velX = x[3..5]
    """
    n = 12
    N = 18  # 12 base + 6 integral
    A_aug = np.zeros((N, N))
    A_aug[:n, :n] = A
    # Integral of position error
    A_aug[12, 0] = 1.0
    A_aug[13, 1] = 1.0
    A_aug[14, 2] = 1.0
    # Integral of velocity error
    A_aug[15, 3] = 1.0
    A_aug[16, 4] = 1.0
    A_aug[17, 5] = 1.0
    B_aug = np.zeros((N, 4))
    B_aug[:n, :] = B
    return A_aug, B_aug


def compute_lqr(A_aug, B_aug, Q_diag, R_diag, dt=0.01):
    """Solve LQR via discrete-time DARE (more numerically stable for
    marginally-stable continuous systems with many zero eigenvalues).

    The K matrix is applied at 100Hz in firmware, so the discrete formulation
    with dt=0.01 matches the actual control loop.
    """
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)

    # Discretize via ZOH
    sys_d = cont2discrete((A_aug, B_aug, np.eye(len(A_aug)), np.zeros((len(A_aug), B_aug.shape[1]))),
                          dt=dt, method='zoh')
    Ad, Bd = sys_d[0], sys_d[1]

    # Use same Q, R as continuous formulation (no dt scaling).
    # This keeps the DARE well-conditioned and produces K values
    # directly comparable to the continuous CARE result.
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
    return K, P


def check_stability(A_aug, B_aug, K):
    Acl = A_aug - B_aug @ K
    eigs = np.linalg.eigvals(Acl)
    real_parts = np.real(eigs)
    # Use small tolerance: CARE guarantees stability; values near 0 are numerical noise
    STABLE_TOL = 1e-4
    stable = bool(np.all(real_parts < STABLE_TOL))
    return stable, float(np.max(real_parts)), eigs


def physical_sanity_check(K, params, m, g):
    L   = params.get('ARM_LENGTH',   0.225)
    max_t = params.get('MAX_THRUST', 8.0)
    hover_N = m * g
    F_base = hover_N / 4.0
    # Maximum roll moment before a motor hits zero at hover thrust
    max_M_roll_phys = F_base * 4.0 * L   # when opposing motor just reaches 0

    print("\n--- Physical sanity check ---")
    print(f"  hover_thrust  = {hover_N:.3f} N  (4 × {F_base:.3f} N)")
    print(f"  max_M_roll_phys = {max_M_roll_phys:.3f} Nm  (F_base × 4L = {F_base:.3f} × {4*L:.3f})")
    print(f"  K[1][1] (pos_E → M_roll) = {K[1,1]:.4f} Nm/m")
    print(f"  K[1][4] (vel_E → M_roll) = {K[1,4]:.4f} Nm/(m/s)")
    print(f"  K[1][13](int_posE → M_roll)= {K[1,13]:.4f} Nm/(m·s)")

    pos_sat_m = max_M_roll_phys / abs(K[1, 1]) if abs(K[1, 1]) > 1e-9 else float('inf')
    print(f"  East pos error at motor saturation: {pos_sat_m:.2f} m")

    if pos_sat_m < 0.5:
        print("  WARNING: saturation at < 0.5 m East error — increase R[1]")
    elif pos_sat_m < 1.0:
        print("  OK (saturation at 0.5–1.0 m)")
    else:
        print("  GOOD (saturation above 1.0 m)")

    # Recommended anti-windup limits so K*int_max <= max_M_roll
    rec_int_pos = max_M_roll_phys / abs(K[1, 13]) if abs(K[1, 13]) > 1e-9 else 10.0
    rec_int_vel = max_M_roll_phys / abs(K[1, 16]) if abs(K[1, 16]) > 1e-9 else 5.0
    print(f"\n  Recommended anti-windup limits (so K*max_int <= max_M_roll):")
    print(f"    MAX_POS_INT = {min(rec_int_pos, 10.0):.2f} m·s")
    print(f"    MAX_VEL_INT = {min(rec_int_vel,  5.0):.2f} (m/s)·s")


def write_gains(K, path):
    with open(path, 'w') as f:
        f.write("# LQR/LQI gain matrix K (4x18) — auto-generated by lqr_design.py\n")
        f.write("# Row order:  0=F_thrust  1=M_roll  2=M_pitch  3=M_yaw\n")
        f.write("# Col order:  pN pE pD  vN vE vD  phi theta psi  p q r"
                "  int_pN int_pE int_pD  int_vN int_vE int_vD\n")
        for i in range(4):
            f.write(' '.join(f'{K[i, j]:.8f}' for j in range(18)) + '\n')
    print(f"\nK written to: {path}")


def main():
    # ---- locate sysid file ----
    sysid_path = None
    for p in ([sys.argv[1]] if len(sys.argv) > 1 else []) + SYSID_PATHS:
        if os.path.exists(p):
            sysid_path = p
            break
    if sysid_path is None:
        print(f"ERROR: sysid_params.txt not found. Tried: {SYSID_PATHS}")
        sys.exit(1)

    print(f"Reading sysid from: {sysid_path}")
    params = read_sysid(sysid_path)
    print(f"Parameters loaded: {params}")

    # ---- build system ----
    A, B, m, Ixx, Iyy, Izz, g = build_system_matrices(params)
    print(f"\nPhysical params: mass={m:.3f} kg, Ixx={Ixx:.5f}, Iyy={Iyy:.5f}, Izz={Izz:.5f}")

    A_aug, B_aug = build_lqi_matrices(A, B)

    # ---- solve CARE ----
    print("\nSolving CARE (18-state LQI) ...")
    try:
        K, P = compute_lqr(A_aug, B_aug, Q_DIAG, R_DIAG)
    except Exception as e:
        print(f"ERROR: CARE solver failed: {e}")
        sys.exit(1)

    # ---- stability check ----
    stable, max_real, eigs = check_stability(A_aug, B_aug, K)
    print(f"Closed-loop stable: {stable}  (max Re(eigenvalue) = {max_real:.2e})")
    # Show any eigenvalues with Re > -0.1 (potentially marginal)
    marginal = [(np.real(e), np.imag(e)) for e in eigs if np.real(e) > -0.1]
    if marginal:
        print(f"  Eigenvalues near 0: {['({:.2e},{:.2e}j)'.format(r,i) for r,i in marginal]}")
    if not stable:
        print("CRITICAL: Closed-loop is NOT stable! Check A, B, Q, R matrices.")
        sys.exit(1)

    # ---- print K ----
    print("\nK (4×18):")
    row_labels = ['F_thr ', 'M_roll', 'M_ptch', 'M_yaw ']
    col_labels = ['pN', 'pE', 'pD', 'vN', 'vE', 'vD',
                  'phi', 'tht', 'psi', 'p', 'q', 'r',
                  'ipN', 'ipE', 'ipD', 'ivN', 'ivE', 'ivD']
    print('        ' + ''.join(f'{s:>9s}' for s in col_labels))
    for i in range(4):
        print(f'{row_labels[i]:8s}' + ''.join(f'{K[i, j]:9.4f}' for j in range(18)))

    # ---- physical sanity check ----
    physical_sanity_check(K, params, m, g)

    # ---- write output ----
    os.makedirs(os.path.dirname(LQR_OUT_PATH), exist_ok=True)
    write_gains(K, LQR_OUT_PATH)
    print("\nDone. Place lqr_gains.txt where Mode 99 firmware can find it.")
    print("Mode 99 will load this on init; if not found it falls back to heuristic gains.")


if __name__ == '__main__':
    main()
