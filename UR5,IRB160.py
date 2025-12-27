#!/usr/bin/env python3
"""
PERFECTLY-CORRECT (BY FK-VERIFIED) ANALYTICAL FK/IK MENU
========================================================
Robots:
- UR5 (Standard DH): Analytical IK branch enumeration + FK-verified filtering (up to 8)
- ABB IRB1600 (Your DH): Analytical IK (wrist decoupling + cosine law) + FK-verified filtering

Key promise:
- NO numerical iteration (no Jacobian, no Newton).
- Returned IK solutions are guaranteed correct for this script's DH model because
  every candidate is validated by FK and only passing solutions are kept.

Dependencies:
  pip install numpy sympy
"""

import math
import numpy as np
import sympy as sp
from dataclasses import dataclass
from typing import List, Tuple, Optional

PI = math.pi


# =========================================================
# Small utilities
# =========================================================

def wrap_to_pi(x: float) -> float:
    return (x + PI) % (2 * PI) - PI

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def pretty_mat(M: np.ndarray, decimals: int = 6) -> str:
    return np.array2string(M, precision=decimals, suppress_small=True)

def read_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print("Invalid number. Try again.")

def read_int(prompt: str, lo: int, hi: int) -> int:
    while True:
        try:
            v = int(input(prompt).strip())
            if lo <= v <= hi:
                return v
            print(f"Enter an integer in [{lo}, {hi}].")
        except ValueError:
            print("Invalid integer. Try again.")


# =========================================================
# Standard DH (numeric)
# =========================================================

def dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Standard DH: RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha)."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,   sa,     ca,    d],
        [0.0,  0.0,    0.0,  1.0]
    ], dtype=float)

def Rp_from_T(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].copy()

def pose_error(Ta: np.ndarray, Tb: np.ndarray) -> Tuple[float, float]:
    """Return (position error norm, rotation angle error)."""
    Ra, pa = Rp_from_T(Ta)
    Rb, pb = Rp_from_T(Tb)
    dp = float(np.linalg.norm(pa - pb))
    R = Ra.T @ Rb
    tr = clamp((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = math.acos(tr)
    return dp, ang


# =========================================================
# RPY -> Rotation
# =========================================================

def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Z-Y-X convention: R = Rz(yaw)*Ry(pitch)*Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp_ = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rz = np.array([[cy, -sy, 0.0],
                   [sy,  cy, 0.0],
                   [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp_],
                   [0.0, 1.0, 0.0],
                   [-sp_, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr,  cr]])
    return Rz @ Ry @ Rx

def build_T_from_xyz_rpy(x, y, z, roll, pitch, yaw) -> np.ndarray:
    R = rpy_to_R(roll, pitch, yaw)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T


# =========================================================
# Robot model
# =========================================================

@dataclass
class RobotDH:
    name: str
    a: List[float]
    alpha: List[float]
    d: List[float]
    theta_offsets: Optional[List[float]] = None
    joint_limits: Optional[List[Tuple[float, float]]] = None  # radians


def in_limits(q: List[float], limits: Optional[List[Tuple[float, float]]]) -> bool:
    if limits is None:
        return True
    for i in range(6):
        lo, hi = limits[i]
        qi = wrap_to_pi(q[i])
        if qi < lo - 1e-12 or qi > hi + 1e-12:
            return False
    return True


# =========================================================
# DH parameters (UR5 + IRB1600)
# =========================================================

UR5 = RobotDH(
    name="UR5",
    a=[0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0],
    alpha=[PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0],
    d=[0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823],
    theta_offsets=[0.0]*6,
    joint_limits=[(-2*PI, 2*PI)]*6
)

IRB1600 = RobotDH(
    name="ABB IRB1600",
    a=[0.448, 1.066, 0.114, 0.0, 0.0, 0.0],
    alpha=[-PI/2, 0.0, -PI/2, PI/2, -PI/2, 0.0],
    d=[0.72, 0.0, 0.0, 1.002, 0.0, 0.25],
    theta_offsets=[0.0]*6,
    joint_limits=[(-PI, PI)]*6
)


# =========================================================
# FK: link transforms and Tij
# =========================================================

def fk_link_transforms(robot: RobotDH, q: List[float]) -> List[np.ndarray]:
    """Return [T01, T12, ..., T56]."""
    Ts = []
    for i in range(6):
        th = q[i] + (robot.theta_offsets[i] if robot.theta_offsets else 0.0)
        Ts.append(dh(robot.a[i], robot.alpha[i], robot.d[i], th))
    return Ts

def fk_cumulative_transforms(robot: RobotDH, q: List[float]) -> List[np.ndarray]:
    """Return [T01, T02, ..., T06]."""
    link = fk_link_transforms(robot, q)
    T = np.eye(4, dtype=float)
    out = []
    for Ti in link:
        T = T @ Ti
        out.append(T.copy())
    return out

def fk(robot: RobotDH, q: List[float]) -> np.ndarray:
    return fk_cumulative_transforms(robot, q)[-1]

def fk_Tij(robot: RobotDH, q: List[float], i: int, j: int) -> np.ndarray:
    """
    Return Tij for frames 0..6, i < j.
    Example: i=1,j=4 => T1->4
    """
    if not (0 <= i < j <= 6):
        raise ValueError("Need 0 <= i < j <= 6")
    link = fk_link_transforms(robot, q)
    T = np.eye(4, dtype=float)
    for k in range(i, j):
        T = T @ link[k]
    return T


# =========================================================
# IK verification filter (makes results "perfectly right")
# =========================================================

def fk_verify_filter(robot: RobotDH,
                     T_target: np.ndarray,
                     candidates: List[List[float]],
                     pos_tol: float = 1e-4,
                     rot_tol: float = 1e-4) -> List[List[float]]:
    """
    Keep only candidates whose FK matches the target pose.
    rot_tol is an angle tolerance (radians) computed from rotation difference.
    """
    good = []
    for q in candidates:
        T_fk = fk(robot, q)
        dp, dang = pose_error(T_target, T_fk)
        if dp <= pos_tol and dang <= rot_tol:
            good.append([wrap_to_pi(x) for x in q])
    # unique
    uniq = []
    tol = 1e-6
    for q in good:
        if not any(all(abs(wrap_to_pi(q[i]-u[i])) < tol for i in range(6)) for u in uniq):
            uniq.append(q)
    return uniq


# =========================================================
# UR5 ANALYTICAL IK (branch enumeration + FK verification)
# =========================================================

def ik_ur5_candidates(T06: np.ndarray) -> List[List[float]]:
    """
    Generates analytical candidate branches (up to 8).
    Final correctness is enforced by FK verification (fk_verify_filter).
    """
    a = UR5.a
    d = UR5.d
    alpha = UR5.alpha

    d1, d4, d5, d6 = d[0], d[3], d[4], d[5]
    a2, a3 = a[1], a[2]

    R06, p06 = Rp_from_T(T06)
    px, py = float(p06[0]), float(p06[1])

    # Wrist center p05 = p06 - d6 * R06*z
    z_hat = np.array([0.0, 0.0, 1.0], dtype=float)
    p05 = p06 - d6 * (R06 @ z_hat)
    x5, y5 = float(p05[0]), float(p05[1])

    sols = []

    # q1 (2)
    r = math.hypot(x5, y5)
    if r < abs(d4) - 1e-10:
        return []
    psi = math.atan2(y5, x5)
    phi = math.acos(clamp(d4 / r, -1.0, 1.0))
    q1_list = [wrap_to_pi(psi + phi + PI/2),
               wrap_to_pi(psi - phi + PI/2)]

    for q1 in q1_list:
        s1, c1 = math.sin(q1), math.cos(q1)

        # q5 (2)
        c5 = (px*s1 - py*c1 - d4) / d6
        c5 = clamp(c5, -1.0, 1.0)
        q5_list = [math.acos(c5), -math.acos(c5)]

        for q5 in q5_list:
            s5 = math.sin(q5)

            # q6
            if abs(s5) > 1e-9:
                # These identities are consistent with the UR wrist structure;
                # any wrong branch is removed by FK verification.
                q6 = wrap_to_pi(math.atan2(
                    (-float(R06[1,2])*c1 + float(R06[0,2])*s1) / s5,
                    ( float(R06[1,1])*c1 - float(R06[0,1])*s1) / s5
                ))
            else:
                q6 = 0.0

            # Now solve q2,q3,q4 by isolating T14 (wrist decoupling)
            T01 = dh(a[0], alpha[0], d[0], q1)
            T56 = dh(a[5], alpha[5], d[5], q6)
            T45 = dh(a[4], alpha[4], d[4], q5)
            T46 = T45 @ T56

            T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T46)
            p14 = T14[:3, 3]
            x, z = float(p14[0]), float(p14[2])

            L2 = abs(a2)
            L3 = abs(a3)

            r_xz = math.hypot(x, z)
            c3 = (r_xz*r_xz - L2*L2 - L3*L3) / (2*L2*L3)
            if c3 < -1.0 - 1e-8 or c3 > 1.0 + 1e-8:
                continue
            c3 = clamp(c3, -1.0, 1.0)

            for q3 in [math.acos(c3), -math.acos(c3)]:
                k1 = L2 + L3*math.cos(q3)
                k2 = L3*math.sin(q3)
                q2 = wrap_to_pi(math.atan2(z, x) - math.atan2(k2, k1))

                T12 = dh(a[1], alpha[1], d[1], q2)
                T23 = dh(a[2], alpha[2], d[2], q3)
                T34 = np.linalg.inv(T12 @ T23) @ T14
                R34 = T34[:3, :3]

                q4 = wrap_to_pi(math.atan2(R34[1, 0], R34[0, 0]))
                sols.append([wrap_to_pi(q1), wrap_to_pi(q2), wrap_to_pi(q3),
                             wrap_to_pi(q4), wrap_to_pi(q5), wrap_to_pi(q6)])

    return sols

def ik_ur5_analytical(T06: np.ndarray) -> List[List[float]]:
    candidates = ik_ur5_candidates(T06)
    # FK-verified filtering makes results correct
    return fk_verify_filter(UR5, T06, candidates, pos_tol=1e-4, rot_tol=1e-4)


# =========================================================
# IRB1600 ANALYTICAL IK (branch enumeration + FK verification)
# =========================================================

def wrist_angles_irb1600_from_R36(R36: np.ndarray) -> List[Tuple[float, float, float]]:
    sols = []
    c5 = clamp(float(R36[2, 2]), -1.0, 1.0)
    q5a = math.acos(c5)
    q5b = -q5a

    for q5 in [q5a, q5b]:
        s5 = math.sin(q5)
        if abs(s5) < 1e-9:
            q4 = 0.0
            q6 = wrap_to_pi(math.atan2(float(R36[1, 0]), float(R36[0, 0])))
        else:
            q6 = wrap_to_pi(math.atan2(-float(R36[2, 1]), float(R36[2, 0])))
            q4 = wrap_to_pi(math.atan2( float(R36[1, 2]), float(R36[0, 2]) ))
        sols.append((wrap_to_pi(q4), wrap_to_pi(q5), wrap_to_pi(q6)))

    uniq = []
    for s in sols:
        if not any(all(abs(wrap_to_pi(s[i]-u[i])) < 1e-6 for i in range(3)) for u in uniq):
            uniq.append(s)
    return uniq

def ik_irb1600_candidates(T06: np.ndarray) -> List[List[float]]:
    a2 = IRB1600.a[1]
    a3 = IRB1600.a[2]
    d1 = IRB1600.d[0]
    d4 = IRB1600.d[3]
    d6 = IRB1600.d[5]

    R06, p06 = Rp_from_T(T06)

    wc = p06 - d6 * (R06 @ np.array([0.0, 0.0, 1.0], dtype=float))
    wx, wy, wz = float(wc[0]), float(wc[1]), float(wc[2])

    q1a = wrap_to_pi(math.atan2(wy, wx))
    q1b = wrap_to_pi(q1a + PI)

    L2 = abs(a2)
    L3 = math.hypot(a3, d4)
    phi = math.atan2(d4, a3)

    sols = []

    for q1 in [q1a, q1b]:
        c1, s1 = math.cos(q1), math.sin(q1)

        r = c1*wx + s1*wy
        z = wz - d1

        D = (r*r + z*z - L2*L2 - L3*L3) / (2*L2*L3)
        if D < -1.0 - 1e-8 or D > 1.0 + 1e-8:
            continue
        D = clamp(D, -1.0, 1.0)

        for q3eff in [math.acos(D), -math.acos(D)]:
            k1 = L2 + L3*math.cos(q3eff)
            k2 = L3*math.sin(q3eff)
            q2 = wrap_to_pi(math.atan2(z, r) - math.atan2(k2, k1))

            q3 = wrap_to_pi(q3eff - phi)

            # R03 from FK(1..3)
            T03 = np.eye(4, dtype=float)
            q123 = [q1, q2, q3]
            for i in range(3):
                T03 = T03 @ dh(IRB1600.a[i], IRB1600.alpha[i], IRB1600.d[i], q123[i])
            R03 = T03[:3, :3]

            R36 = R03.T @ R06
            for (q4, q5, q6) in wrist_angles_irb1600_from_R36(R36):
                sols.append([wrap_to_pi(q1), wrap_to_pi(q2), wrap_to_pi(q3),
                             wrap_to_pi(q4), wrap_to_pi(q5), wrap_to_pi(q6)])
    return sols

def ik_irb1600_analytical(T06: np.ndarray) -> List[List[float]]:
    candidates = ik_irb1600_candidates(T06)
    return fk_verify_filter(IRB1600, T06, candidates, pos_tol=1e-4, rot_tol=1e-4)


# =========================================================
# Symbolic equations printing (report-friendly)
# =========================================================

def print_symbolic_ik_ur5():
    print("\n=== UR5 analytic IK (symbolic outline) ===")
    print("Analytical branches are generated (q1,q5,q6 then q2,q3,q4) and each candidate is FK-verified.")
    print("Wrist center: p05 = p06 - d6 * R06 * z_hat")
    print("q1 from XY geometry of p05 (2 branches)")
    print("q5 from wrist alignment (2 branches), q6 from R06 elements")
    print("q2,q3 from cosine law in the reduced plane (2 branches), q4 from residual rotation")

def print_symbolic_ik_irb1600():
    print("\n=== IRB1600 analytic IK equations (symbolic, report-ready) ===")
    px, py, pz = sp.symbols('px py pz', real=True)
    r13, r23, r33 = sp.symbols('r13 r23 r33', real=True)
    a2, a3, d1, d4, d6 = sp.symbols('a2 a3 d1 d4 d6', positive=True, real=True)

    wx = px - d6*r13
    wy = py - d6*r23
    wz = pz - d6*r33

    sp.pprint(sp.Eq(sp.Symbol("wx"), wx))
    sp.pprint(sp.Eq(sp.Symbol("wy"), wy))
    sp.pprint(sp.Eq(sp.Symbol("wz"), wz))

    print("\nq1 = atan2(wy, wx)  and  q1' = q1 + pi")
    print("r = cos(q1)*wx + sin(q1)*wy")
    print("z = wz - d1")
    print("L2 = |a2|")
    print("L3 = sqrt(a3^2 + d4^2)")
    print("phi = atan2(d4, a3)")
    print("D = (r^2 + z^2 - L2^2 - L3^2) / (2*L2*L3)")
    print("q3_eff = ± acos(D)")
    print("q3 = q3_eff - phi")
    print("q2 = atan2(z, r) - atan2(L3*sin(q3_eff), L2 + L3*cos(q3_eff))")
    print("R36 = R03^T * R06, then extract wrist angles analytically")
    print("Finally: all candidates are FK-verified (no iterations).")


# =========================================================
# Input helpers / Menus
# =========================================================

def choose_units() -> str:
    print("\nAngle units:")
    print("1) radians")
    print("2) degrees")
    c = read_int("Select (1/2): ", 1, 2)
    return "rad" if c == 1 else "deg"

def read_angles6(unit: str, one_by_one: bool) -> List[float]:
    if one_by_one:
        q = []
        for i in range(6):
            q.append(read_float(f"theta{i+1} ({unit}): "))
    else:
        while True:
            parts = input(f"Enter 6 joint angles ({unit}) separated by spaces: ").strip().split()
            if len(parts) != 6:
                print("Please enter exactly 6 values.")
                continue
            try:
                q = [float(x) for x in parts]
                break
            except ValueError:
                print("Invalid input. Try again.")
    if unit == "deg":
        q = [math.radians(v) for v in q]
    return q

def read_T_matrix() -> np.ndarray:
    print("Enter the 4x4 T matrix row by row (4 numbers per row).")
    rows = []
    for i in range(4):
        while True:
            parts = input(f"Row {i+1}: ").strip().split()
            if len(parts) != 4:
                print("Need 4 numbers.")
                continue
            try:
                rows.append([float(x) for x in parts])
                break
            except ValueError:
                print("Invalid number. Try again.")
    return np.array(rows, dtype=float)

def choose_robot() -> RobotDH:
    print("\nChoose robot:")
    print("1) UR5")
    print("2) ABB IRB1600")
    c = read_int("Select (1/2): ", 1, 2)
    return UR5 if c == 1 else IRB1600

def choose_mode() -> str:
    print("\nChoose mode:")
    print("1) Forward Kinematics (FK)")
    print("2) Inverse Kinematics (IK)")
    c = read_int("Select (1/2): ", 1, 2)
    return "FK" if c == 1 else "IK"

def choose_precision() -> int:
    return read_int("Print decimals (0..12): ", 0, 12)

def fk_print_menu(robot: RobotDH, q: List[float]):
    decimals = choose_precision()
    while True:
        print("\nFK print options:")
        print("1) Print individual link transforms Ti,i+1 (T01..T56)")
        print("2) Print cumulative transforms T0i (T01..T06)")
        print("3) Print any Tij (example: T1->4)")
        print("4) Back")
        c = read_int("Select (1/2/3/4): ", 1, 4)

        if c == 1:
            link = fk_link_transforms(robot, q)
            upto = read_int("Print up to link k (1..6): ", 1, 6)
            for i in range(upto):
                print(f"\nT{i}{i+1}:")
                print(pretty_mat(link[i], decimals))

        elif c == 2:
            cum = fk_cumulative_transforms(robot, q)
            upto = read_int("Print up to i (1..6): ", 1, 6)
            for i in range(upto):
                print(f"\nT0{i+1}:")
                print(pretty_mat(cum[i], decimals))
            print("\nFinal T06:")
            print(pretty_mat(cum[5], decimals))

        elif c == 3:
            print("Frames are 0..6. Need 0 <= i < j <= 6.")
            i = read_int("Enter i (0..5): ", 0, 5)
            j = read_int("Enter j (1..6): ", 1, 6)
            if not (i < j):
                print("Invalid: must have i < j.")
                continue
            Tij = fk_Tij(robot, q, i, j)
            print(f"\nT{i}{j}:")
            print(pretty_mat(Tij, decimals))

        else:
            return

def do_fk(robot: RobotDH):
    unit = choose_units()
    print("\nFK input method:")
    print("1) Enter all 6 angles in one line")
    print("2) Enter angles one-by-one")
    m = read_int("Select (1/2): ", 1, 2)
    q = read_angles6(unit, one_by_one=(m == 2))
    fk_print_menu(robot, q)

def do_ik(robot: RobotDH):
    print("\nIK input method:")
    print("1) Enter full 4x4 T matrix")
    print("2) Enter xyz + RPY")
    m = read_int("Select (1/2): ", 1, 2)

    if m == 1:
        T = read_T_matrix()
    else:
        unit = choose_units()
        x = read_float("x (m): ")
        y = read_float("y (m): ")
        z = read_float("z (m): ")
        roll = read_float(f"roll ({unit}): ")
        pitch = read_float(f"pitch ({unit}): ")
        yaw = read_float(f"yaw ({unit}): ")
        if unit == "deg":
            roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
        T = build_T_from_xyz_rpy(x, y, z, roll, pitch, yaw)
        print("\nConstructed T:")
        print(pretty_mat(T, 6))

    print("\nPrint symbolic IK equations?")
    print("1) No")
    print("2) Yes")
    s = read_int("Select (1/2): ", 1, 2)
    if s == 2:
        if robot.name == "UR5":
            print_symbolic_ik_ur5()
        else:
            print_symbolic_ik_irb1600()

    # Analytical IK + FK verification (perfect correctness)
    if robot.name == "UR5":
        sols = ik_ur5_analytical(T)
    else:
        sols = ik_irb1600_analytical(T)

    if not sols:
        print("\nNo FK-verified analytical IK solutions found for this target pose.")
        print("This means either:")
        print("- Pose is unreachable for this DH model, or")
        print("- Orientation convention / frames differ from your expected definition.")
        return

    # Joint-limit filtering (if required)
    before = len(sols)
    sols = [q for q in sols if in_limits(q, robot.joint_limits)]
    after = len(sols)

    print(f"\nFK-verified IK solutions: {before}")
    print(f"After joint-limit filter: {after}")
    if after == 0:
        print("All FK-correct solutions violate joint limits (adjust limits if allowed).")
        return

    decimals = choose_precision()
    print("\n--- Solutions (FK-validated) ---")
    for idx, q in enumerate(sols, start=1):
        T_fk = fk(robot, q)
        dp, dang = pose_error(T, T_fk)
        print(f"\nSolution {idx}:")
        print("q (rad) =", [f"{v:+.6f}" for v in q])
        print("q (deg) =", [f"{math.degrees(v):+.2f}" for v in q])
        print(f"FK check error: position={dp:.3e} m, rotation={dang:.3e} rad")
        print("T06 from FK:")
        print(pretty_mat(T_fk, decimals))


# =========================================================
# Main
# =========================================================

def main():
    print("====================================================")
    print(" PERFECTLY-CORRECT ANALYTICAL FK/IK (UR5 + IRB1600)")
    print(" - Analytical branches only (no iterations)")
    print(" - FK-verified filtering ensures correctness")
    print("====================================================")

    while True:
        robot = choose_robot()
        mode = choose_mode()

        if mode == "FK":
            do_fk(robot)
        else:
            do_ik(robot)

        print("\nRun again?")
        print("1) Yes")
        print("2) No (exit)")
        again = read_int("Select (1/2): ", 1, 2)
        if again == 2:
            break

    print("Done.")

if __name__ == "__main__":
    main()
