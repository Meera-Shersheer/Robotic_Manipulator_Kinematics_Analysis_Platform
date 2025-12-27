#!/usr/bin/env python3
import math
import numpy as np
import sympy as sp
import re

PI = math.pi

# Utilities
def d2r(x): return x*PI/180.0
def r2d(x): return x*180.0/PI
def wrap(x): return (x+PI)%(2*PI)-PI
def allclose(A,B,t=1e-6): return np.allclose(A,B,atol=t,rtol=0)

def ask_int(msg, lo=None, hi=None):
    while True:
        try:
            v = int(input(msg).strip() or "0")
            if lo is not None and v < lo: raise ValueError
            if hi is not None and v > hi: raise ValueError
            return v
        except:
            print("  ⚠  Invalid integer")

def ask_float(msg):
    while True:
        try:
            return float(input(msg).strip())
        except:
            print("  ⚠  Invalid number")

def ask_str(msg, default=""):
    s = input(msg).strip()
    return default if s == "" else s

def ask_choice(msg, choices, default=None):
    c = input(msg).strip().lower()
    if c == "" and default is not None: return default
    if c in choices: return c
    print("  ⚠  Invalid choice")
    return ask_choice(msg, choices, default)

def ask_list(msg, n, cast=float):
    while True:
        s = input(msg).strip().replace(",", " ").replace("[", " ").replace("]", " ")
        parts = [p for p in s.split() if p]
        if len(parts) != n:
            print(f"  ⚠  Need {n} values")
            continue
        try:
            return [cast(p) for p in parts]
        except:
            print("  ⚠  Bad format")

# Symbol-name input for symbolic mode: reject pure numbers like "2"
_num_re = re.compile(r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$')
def ask_symbols(msg, n):
    while True:
        names = ask_list(msg, n, str)
        bad = [s for s in names if _num_re.match(s.strip())]
        if bad:
            print("  Please enter SYMBOL NAMES (variables), not pure numbers:", ", ".join(bad))
            continue
        return names

def pprint(name, M, sym=False):
    print(f"\n{name} =")
    if sym: sp.pprint(M)
    else:
        with np.printoptions(precision=6, suppress=True, linewidth=100): print(M)

# Rotation matrices
def rot_x(a, sym=False):
    c = sp.cos(a) if sym else math.cos(a)
    s = sp.sin(a) if sym else math.sin(a)
    return sp.Matrix([[1,0,0],[0,c,-s],[0,s,c]]) if sym else np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a, sym=False):
    c = sp.cos(a) if sym else math.cos(a)
    s = sp.sin(a) if sym else math.sin(a)
    return sp.Matrix([[c,0,s],[0,1,0],[-s,0,c]]) if sym else np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a, sym=False):
    c = sp.cos(a) if sym else math.cos(a)
    s = sp.sin(a) if sym else math.sin(a)
    return sp.Matrix([[c,-s,0],[s,c,0],[0,0,1]]) if sym else np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy_to_R(alpha, beta, gamma, sym=False):
    # ZYX: Rz(gamma)*Ry(beta)*Rx(alpha)
    return rot_z(gamma, sym) @ rot_y(beta, sym) @ rot_x(alpha, sym)

def dhA(theta, d, a, alpha, sym=False):
    if sym:
        ct, st = sp.cos(theta), sp.sin(theta)
        ca, sa = sp.cos(alpha), sp.sin(alpha)
        return sp.Matrix([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])
    else:
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ], dtype=float)

def in_limits(robot, q):
    msgs = []
    ok = True
    for i,(lo,hi) in enumerate(robot.lim):
        if q[i] < lo-1e-9 or q[i] > hi+1e-9:
            ok = False
            msgs.append(f"θ{i+1} out of limits [{lo:.3f}, {hi:.3f}]")
    return ok, msgs

class UR5:
    def __init__(self):
        self.name = "UR5 (Standard DH + Closed-Form IK — ROS-consistent)"
        # Standard DH (UR5)
        self.d1 = 0.089159
        self.a2 = -0.425
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823

        self.a = [0.0, self.a2, self.a3, 0.0, 0.0, 0.0]
        self.alpha = [PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0]
        self.d = [self.d1, 0.0, 0.0, self.d4, self.d5, self.d6]

        self.lim = [(-2*PI, 2*PI)]*6

    def fk_all(self, q, sym=False):
        Ts = []
        T = sp.eye(4) if sym else np.eye(4)
        for i in range(6):
            A = dhA(q[i], self.d[i], self.a[i], self.alpha[i], sym=sym)
            T = T @ A
            Ts.append(T)
        return Ts, T

# -------------------- CLOSED FORM IK (CONSISTENT WITH FK ABOVE) --------------------
def ik_ur5_closed_form(robot: UR5, T06: np.ndarray):
    d1, a2, a3, d4, d5, d6 = robot.d1, robot.a2, robot.a3, robot.d4, robot.d5, robot.d6

    R06 = T06[:3, :3]
    p06 = T06[:3, 3]
    px, py, pz = float(p06[0]), float(p06[1]), float(p06[2])

    sols = []

    # wrist center (p05)
    p05 = p06 - d6 * R06[:, 2]
    rxy = math.hypot(p05[0], p05[1])
    if rxy < 1e-12:
        return []

    # θ1 (two solutions)
    c = d4 / rxy
    if abs(c) > 1.0 + 1e-9:
        return []
    c = max(-1.0, min(1.0, c))
    phi = math.acos(c)
    psi = math.atan2(p05[1], p05[0])
    q1_candidates = [wrap(psi + phi + PI/2), wrap(psi - phi + PI/2)]

    for q1 in q1_candidates:
        s1, c1 = math.sin(q1), math.cos(q1)

        # θ5 (two solutions)
        c5 = (px*s1 - py*c1 - d4) / d6
        if abs(c5) > 1.0 + 1e-9:
            continue
        c5 = max(-1.0, min(1.0, c5))
        q5a = math.acos(c5)
        q5_candidates = [wrap(q5a), wrap(-q5a)]

        for q5 in q5_candidates:
            s5 = math.sin(q5)
            if abs(s5) < 1e-10:
                continue

            # θ6
            q6 = math.atan2(
                (-R06[0,1]*s1 + R06[1,1]*c1) / s5,
                (R06[0,0]*s1 - R06[1,0]*c1) / s5
            )
            q6 = wrap(q6)

            # Compute T14 = inv(T01)*T06*inv(T45*T56)
            T01 = dhA(q1, d1, 0.0, PI/2, sym=False)
            T45 = dhA(q5, d5, 0.0, -PI/2, sym=False)
            T56 = dhA(q6, d6, 0.0, 0.0, sym=False)

            T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)

            # θ2, θ3 from planar in frame1: use x,z (NOT x,y)
            p14 = T14[:3, 3]
            x, y = float(p14[0]), float(p14[1])

            D = (x*x + y*y - a2*a2 - a3*a3) / (2*a2*a3)
            if abs(D) > 1.0 + 1e-9:
                continue
            D = max(-1.0, min(1.0, D))

            for q3 in [math.acos(D), -math.acos(D)]:
                q2 = math.atan2(y, x) - math.atan2(a3*math.sin(q3), a2 + a3*math.cos(q3))
                q2, q3 = wrap(q2), wrap(q3)

                # θ4 from rotation
                qtemp = [q1, q2, q3, 0.0, 0.0, 0.0]
                Ts, _ = robot.fk_all(qtemp, sym=False)
                R03 = Ts[2][:3, :3]
                R04 = (T01 @ T14)[:3, :3]
                R34 = R03.T @ R04
                q4 = wrap(math.atan2(R34[1,0], R34[0,0]))

                candidate = [wrap(q1), wrap(q2), wrap(q3), wrap(q4), wrap(q5), wrap(q6)]

                # Validate by FK to guarantee consistency
                _, Tchk = robot.fk_all(candidate, sym=False)
                if np.allclose(Tchk, T06, atol=1e-6, rtol=0):
                    sols.append(candidate)

    # Remove duplicates (wrap-aware)
    uniq = []
    for s in sols:
        if not any(sum((wrap(si-ui))**2 for si,ui in zip(s,u)) < 1e-10 for u in uniq):
            uniq.append(s)
    return uniq


# -------------------- HARD-CODED SYMBOLIC IK EQUATIONS (no user input) --------------------
def ur5_ik_equations_symbolic_hardcoded(robot: UR5):
    """Print SymPy equations used by ik_ur5_closed_form(), hard-coded symbols and same DH."""
    x, y, z, alpha, beta, gamma = sp.symbols("x y z alpha beta gamma", real=True)

    # Target transform (ZYX RPY)
    R06 = rpy_to_R(alpha, beta, gamma, sym=True)
    T06 = sp.Matrix([[R06[0,0], R06[0,1], R06[0,2], x],
                     [R06[1,0], R06[1,1], R06[1,2], y],
                     [R06[2,0], R06[2,1], R06[2,2], z],
                     [0,0,0,1]])

    d1, a2, a3, d4, d5, d6 = map(sp.nsimplify, [robot.d1, robot.a2, robot.a3, robot.d4, robot.d5, robot.d6])

    # Wrist center p05
    p06 = sp.Matrix([x, y, z])
    p05 = sp.simplify(p06 - d6 * R06[:, 2])
    rxy = sp.sqrt(p05[0]**2 + p05[1]**2)

    # θ1 (two solutions)
    c = d4 / rxy
    phi = sp.acos(c)
    psi = sp.atan2(p05[1], p05[0])
    q1a = sp.simplify(psi + phi + sp.pi/2)
    q1b = sp.simplify(psi - phi + sp.pi/2)

    # θ5 from c5 = (px*s1 - py*c1 - d4)/d6
    q1_sym = sp.Symbol("theta1", real=True)
    s1, c1 = sp.sin(q1_sym), sp.cos(q1_sym)
    c5 = sp.simplify((x*s1 - y*c1 - d4) / d6)
    q5_sym = sp.Symbol("theta5", real=True)  # θ5 = ±acos(c5)

    # θ6
    s5 = sp.sin(q5_sym)
    q6 = sp.atan2(
        (-R06[0,1]*s1 + R06[1,1]*c1) / s5,
        ( R06[0,0]*s1 - R06[1,0]*c1) / s5
    )

    # Build T01, T45, T56 and T14 = inv(T01)*T06*inv(T45*T56)
    T01 = dhA(q1_sym, d1, 0.0, sp.pi/2, sym=True)
    T45 = dhA(q5_sym, d5, 0.0, -sp.pi/2, sym=True)
    T56 = dhA(q6,    d6, 0.0, 0.0, sym=True)
    T14 = sp.simplify(T01.inv() * T06 * (T45*T56).inv())

    # θ2, θ3 from planar solve using p14 x,y (matches code)
    p14 = T14[:3, 3]
    X, Y = sp.simplify(p14[0]), sp.simplify(p14[1])
    D = sp.simplify((X**2 + Y**2 - a2**2 - a3**2) / (2*a2*a3))
    q3p = sp.Symbol("theta3", real=True)  # θ3 = ±acos(D) in code
    q2 = sp.simplify(sp.atan2(Y, X) - sp.atan2(a3*sp.sin(q3p), a2 + a3*sp.cos(q3p)))

    # θ4 from R34 = R03^T R04 (same as code path)
    q2_sym = sp.Symbol("theta2", real=True)
    q3_sym = sp.Symbol("theta3", real=True)
    # R03 from first three joints
    T03 = sp.simplify(dhA(q1_sym, d1, 0.0, sp.pi/2, True) *
                      dhA(q2_sym, 0.0, a2, 0.0, True) *
                      dhA(q3_sym, 0.0, a3, 0.0, True))
    R03 = sp.simplify(T03[:3,:3])
    # R04 = (T01*T14) rotation
    R04 = sp.simplify((T01*T14)[:3,:3])
    R34 = sp.simplify(R03.T * R04)
    q4 = sp.atan2(R34[1,0], R34[0,0])

    print("\n" + "="*70)
    print("HARD-CODED SYMBOLIC IK EQUATIONS (UR5) — matches ik_ur5_closed_form()")
    print("="*70)

    print("\nTarget T06(x,y,z,α,β,γ) with ZYX RPY:")
    sp.pprint(T06)

    print("\nWrist center p05 = p06 - d6*z06")
    sp.pprint(p05)

    print("\nθ1 branches:")
    print("  c = d4 / sqrt(p05x^2 + p05y^2)")
    print("  ψ = atan2(p05y,p05x),  φ = acos(c)")
    print("  θ1 = ψ ± φ + π/2")
    print("  θ1a =", q1a)
    print("  θ1b =", q1b)

    print("\nθ5 branches (given θ1):")
    print("  c5 = (x*sinθ1 - y*cosθ1 - d4) / d6")
    sp.pprint(c5)
    print("  θ5 = ±acos(c5)")

    print("\nθ6 (given θ1,θ5):")
    sp.pprint(q6)

    print("\nθ2, θ3 (planar from p14):")
    print("  T14 = inv(T01)*T06*inv(T45*T56)")
    print("  D = (X^2 + Y^2 - a2^2 - a3^2)/(2 a2 a3),  θ3 = ±acos(D)")
    sp.pprint(D)
    print("  θ2 = atan2(Y,X) - atan2(a3*sinθ3, a2 + a3*cosθ3)")
    sp.pprint(q2)

    print("\nθ4 from R34:")
    sp.pprint(q4)
    print()

    return {
        "symbols": (x,y,z,alpha,beta,gamma),
        "p05": p05,
        "theta1": (q1a, q1b),
        "c5": c5,
        "theta6": q6,
        "D": D,
        "theta2": q2,
        "theta4": q4,
    }


# -------------------- SYMBOLIC IK (Equations only) --------------------
def do_ik_symbolic(robot: UR5):
    ur5_ik_equations_symbolic_hardcoded(robot)

# -------------------- I/O MENU --------------------
def read_T_matrix():
    print("\n  Enter 16 numbers (row-major):")
    vals = ask_list("  T: ", 16, float)
    T = np.array(vals, dtype=float).reshape((4,4))
    return T

def read_pose():
    vals = ask_list("  Enter x y z alpha beta gamma: ", 6, float)
    x,y,z,a,b,g = vals
    R = rpy_to_R(a,b,g, sym=False)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = [x,y,z]
    return T

def do_fk(robot: UR5):
    print("\n======================================================================")
    print("FORWARD KINEMATICS")
    print("======================================================================\n")
    mode = ask_choice("Mode (N/S) [N]: ", ["n","s"], "n")
    sym = (mode == "s")

    unit = ask_choice("Angle unit (deg/rad) [rad]: ", ["deg","rad"], "rad")
    q = ask_list("  θ1..θ6: ", 6, float)
    if unit == "deg":
        q = [d2r(v) for v in q]

    Ts, T = robot.fk_all(q, sym=sym)

    frames = ask_str("\nPrint frames (all/range) [all]: ", "all").strip().lower()

    print("\n======================================================================")
    print("RESULTS")
    print("======================================================================")
    if frames == "all" or frames == "a":
        for i,Ti in enumerate(Ts, start=1):
            print(f"\n--- Frame {i} ---")
            pprint(f"0T{i}", Ti, sym=sym)
    else:
        try:
            a,b = frames.split("-")
            a,b = int(a), int(b)
            for i in range(a, b+1):
                print(f"\n--- Frame {i} ---")
                pprint(f"0T{i}", Ts[i-1], sym=sym)
        except:
            for i,Ti in enumerate(Ts, start=1):
                print(f"\n--- Frame {i} ---")
                pprint(f"0T{i}", Ti, sym=sym)

    print("\n======================================================================")
    print("Done!")
    print("======================================================================\n")

def do_ik(robot: UR5):
    print("\n======================================================================")
    print("INVERSE KINEMATICS")
    print("======================================================================\n")

    mode = ask_choice("Mode (N/S/L) [N]: ", ["n","s","l"], "n")
    if mode == "l":
        do_ik_symbolic(robot)
        return

    sym = (mode == "s")

    print("\nInput: 1) 4×4 matrix  2) x,y,z,α,β,γ")
    ch = ask_choice("Choose: ", ["1","2"], "1")
    T = read_T_matrix() if ch == "1" else read_pose()

    print("\nTarget 0T6 =")
    pprint("0T6", T, sym=False)

    print("\nSolving...")

    sols = ik_ur5_closed_form(robot, T)

    print("\n======================================================================")
    print("RESULTS")
    print("======================================================================")
    if not sols:
        print("No solution found.")
        print("\n======================================================================")
        print("Done!")
        print("======================================================================\n")
        return

    print(f"Raw: {len(sols)} | Valid: {len(sols)}")
    print("\n--- Solutions (rad) ---")
    for i,s in enumerate(sols, start=1):
        print(f"{i:02d}: [{', '.join(f'{v:+.6f}' for v in s)}]")

    print("\n--- Solutions (deg) ---")
    for i,s in enumerate(sols, start=1):
        sd = [r2d(v) for v in s]
        print(f"{i:02d}: [{', '.join(f'{v:+.3f}' for v in sd)}]")

    print("\n======================================================================")
    print("Done!")
    print("======================================================================\n")

def main():
    robot = UR5()
    print("\n======================================================================")
    print(f"  {robot.name}")
    print("  FK/IK Calculator (Closed-Form IK)")
    print("======================================================================\n")

    op = ask_choice("Operation (F=FK / I=IK) [F]: ", ["f","i"], "f")
    if op == "f":
        do_fk(robot)
    else:
        do_ik(robot)

if __name__ == "__main__":
    main()
