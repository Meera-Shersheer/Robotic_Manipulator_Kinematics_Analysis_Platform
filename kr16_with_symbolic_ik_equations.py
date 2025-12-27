#!/usr/bin/env python3
"""
KUKA KR16 Robot - Corrected Forward and Inverse Kinematics
Uses Standard DH Parameters with Spherical Wrist Analytical IK
"""
import math
import numpy as np
import sympy as sp

PI = math.pi

# ============================================================================
# Utilities
# ============================================================================
def d2r(x): return x*PI/180.0
def r2d(x): return x*180.0/PI
def wrap(x): return (x+PI)%(2*PI)-PI
def allclose(A,B,t=1e-6): return np.allclose(A,B,atol=t,rtol=0)

def ask_int(msg, lo=None, hi=None):
    while True:
        try:
            v=int(input(msg).strip())
            if lo and v<lo: print(f"  ⚠ >= {lo}"); continue
            if hi and v>hi: print(f"  ⚠ <= {hi}"); continue
            return v
        except: print("  ⚠ Invalid integer")

def ask_list(msg,n,cast=float):
    while True:
        s=input(msg).strip().replace(","," ")
        p=s.split()
        if len(p)!=n: print(f"  ⚠ Need {n} values"); continue
        try: return [cast(x) for x in p]
        except: print("  ⚠ Invalid format")

def pprint(name, M, sym=False):
    print(f"\n{name} =")
    if sym: sp.pprint(M)
    else:
        with np.printoptions(precision=6, suppress=True, linewidth=100): print(M)

# ============================================================================
# Rotation matrices
# ============================================================================
def Rx(a): ca,sa=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
def Ry(b): cb,sb=math.cos(b),math.sin(b); return np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
def Rz(g): cg,sg=math.cos(g),math.sin(g); return np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])

def eulerR(a,b,g,order="ZYX"):
    order=order.upper()
    if order=="ZYX": return Rz(g)@Ry(b)@Rx(a)
    if order=="XYZ": return Rx(a)@Ry(b)@Rz(g)
    raise ValueError("Use ZYX or XYZ")

def Rx_s(a): ca,sa=sp.cos(a),sp.sin(a); return sp.Matrix([[1,0,0],[0,ca,-sa],[0,sa,ca]])
def Ry_s(b): cb,sb=sp.cos(b),sp.sin(b); return sp.Matrix([[cb,0,sb],[0,1,0],[-sb,0,cb]])
def Rz_s(g): cg,sg=sp.cos(g),sp.sin(g); return sp.Matrix([[cg,-sg,0],[sg,cg,0],[0,0,1]])

def eulerR_s(a,b,g,order="ZYX"):
    order=order.upper()
    if order=="ZYX": return sp.simplify(Rz_s(g)*Ry_s(b)*Rx_s(a))
    if order=="XYZ": return sp.simplify(Rx_s(a)*Ry_s(b)*Rz_s(g))
    raise ValueError("Use ZYX or XYZ")

# ============================================================================
# DH transformation
# ============================================================================
def dhA(th,d,a,al,sym=False):
    if sym:
        ct,st,ca,sa=sp.cos(th),sp.sin(th),sp.cos(al),sp.sin(al)
        return sp.Matrix([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
    ct,st,ca,sa=math.cos(th),math.sin(th),math.cos(al),math.sin(al)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0.0,sa,ca,d],[0.0,0.0,0.0,1.0]])

# ============================================================================
# Robot class
# ============================================================================
class KR16:
    def __init__(self):
        self.name="KUKA KR16"
        # Standard DH Parameters (CORRECTED)
        self.a=[0.260, 0.680, 0.035, 0.0, 0.0, 0.0]
        self.alpha=[PI/2, 0.0, PI/2, -PI/2, PI/2, 0.0]
        self.d=[0.675, 0.0, 0.0, 0.67, 0.0, 0.115]
        self.lim=[(d2r(-185),d2r(185)),(d2r(-35),d2r(158)),(d2r(-120),d2r(158)),
                  (d2r(-350),d2r(350)),(d2r(-130),d2r(130)),(d2r(-350),d2r(350))]
    
    def fk_all(self,q,sym=False):
        T=sp.eye(4) if sym else np.eye(4)
        Ts=[]
        for i in range(6):
            T=sp.simplify(T*dhA(q[i],self.d[i],self.a[i],self.alpha[i],True)) if sym else T@dhA(q[i],self.d[i],self.a[i],self.alpha[i],False)
            Ts.append(T if sym else T.copy())
        return Ts, Ts[-1]

    def fk(self, q, sym: bool=False):
        """Convenience: return only the final end-effector transform."""
        return self.fk_all(q, sym)[1]


# ============================================================================
# Validation functions
# ============================================================================
def in_limits(robot,q):
    violations=[]
    for i,(lo,hi) in enumerate(robot.lim):
        if q[i]<lo-1e-9 or q[i]>hi+1e-9:
            violations.append((i+1,r2d(q[i]),r2d(lo),r2d(hi)))
    return len(violations)==0, violations

def wrist_sing(q): return abs(math.sin(q[4]))<1e-6

def fk_verify(robot,q,Tt,tol=1e-4):
    _,T=robot.fk_all(q,False)
    return allclose(T,Tt,tol)

# ============================================================================
# CORRECTED Inverse Kinematics for KR16
# ============================================================================
def ik_kr16(robot,T06, tol=1e-5):
    """
    Analytical IK for KUKA KR16 with spherical wrist (standard DH family).
    Returns unique solutions (up to 8) that pass FK verification.
    DH pattern assumed (as in this script):
      a=[a1,a2,a3,0,0,0], d=[d1,0,0,d4,0,d6]
      alpha=[±pi/2,0,±pi/2,∓pi/2,±pi/2,0]
    """
    a,d,al = robot.a, robot.d, robot.alpha
    R06, p06 = T06[:3,:3], T06[:3,3]

    # Wrist center (frame 5 origin)
    d6 = d[5]
    pwc = p06 - d6 * R06[:,2]

    a1,a2,a3 = a[0], a[1], a[2]
    d1,d4 = d[0], d[3]

    # q1 candidates (account for shoulder offset a1)
    xw,yw,zw = pwc.tolist()
    r_xy = math.hypot(xw, yw)
    th1_list = []
    if abs(a1) < 1e-10:
        th1_list = [math.atan2(yw, xw)]
    else:
        if r_xy < abs(a1) - 1e-9:
            return []
        phi = math.atan2(yw, xw)
        s = math.sqrt(max(0.0, r_xy*r_xy - a1*a1))
        # Two geometric solutions
        th1_list = [phi - math.atan2(a1,  s),
                    phi - math.atan2(a1, -s)]

    sols = []
    # Precompute helpers
    L2 = abs(a2)
    L3 = math.sqrt(a3*a3 + d4*d4)
    gamma = math.atan2(d4, a3)  # elbow offset between a3 and d4

    for th1 in th1_list:
        # Transform wrist center into frame1
        T01 = dhA(th1, d1, a1, al[0], False)
        p1 = np.linalg.inv(T01) @ np.array([xw,yw,zw,1.0])
        x1, y1, z1 = p1[0], p1[1], p1[2]

        # For this DH family, the 2R "arm" lies in the x1-z1 plane
        r = math.hypot(x1, z1)

        # Reachability (with tolerance)
        if r > (L2 + L3) + 1e-8 or r < abs(L2 - L3) - 1e-8:
            continue

        cos_th3p = (r*r - L2*L2 - L3*L3) / (2*L2*L3)
        cos_th3p = max(-1.0, min(1.0, cos_th3p))

        for th3p in [math.acos(cos_th3p), -math.acos(cos_th3p)]:
            th3 = th3p - gamma

            phi = math.atan2(z1, x1)
            psi = math.atan2(L3*math.sin(th3p), L2 + L3*math.cos(th3p))
            th2 = phi - psi

            # Wrist orientation
            qtemp = [th1, th2, th3, 0.0, 0.0, 0.0]
            Ts, _ = robot.fk_all(qtemp, False)
            R03 = Ts[2][:3,:3]
            R36 = R03.T @ R06

            # th5 from R36[2,2], th4/th6 from remaining terms
            c5 = max(-1.0, min(1.0, R36[2,2]))
            th5_base = math.acos(c5)
            for th5 in [th5_base, -th5_base]:
                s5 = math.sin(th5)
                if abs(s5) < 1e-10:
                    th4 = wrap(math.atan2(-R36[1,0], R36[0,0]))
                    th6 = 0.0
                else:
                    th4 = wrap(math.atan2(R36[1,2]/s5, R36[0,2]/s5))
                    th6 = wrap(math.atan2(R36[2,1]/s5, -R36[2,0]/s5))

                q = [wrap(th1), wrap(th2), wrap(th3), wrap(th4), wrap(th5), wrap(th6)]

                # FK verification
                Tchk = robot.fk(q, False)
                if np.allclose(Tchk, T06, atol=tol, rtol=0):
                    # uniqueness (wrap)
                    if not any(np.allclose(np.array(q), np.array(q2), atol=1e-4, rtol=0) for q2 in sols):
                        sols.append(q)
    return sols

def read_T():
    print("\n  Enter 16 numbers (row-major):")
    v=ask_list("  T: ",16,float)
    return np.array(v,float).reshape(4,4)

def read_pose():
    print("\n  Enter position and orientation:")
    v=ask_list("  x y z α β γ: ",6,float)
    x,y,z,al,be,ga=v
    unit=(input("  Angle unit (deg/rad) [deg]: ").strip().lower() or "deg")
    if unit.startswith("d"): al,be,ga=d2r(al),d2r(be),d2r(ga)
    order=(input("  Euler order (ZYX/XYZ) [ZYX]: ").strip().upper() or "ZYX")
    R=eulerR(al,be,ga,order)
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]
    return T,order

# ============================================================================
# Forward kinematics
# ============================================================================
def do_fk(robot):
    print("\n"+"="*70+"\nFORWARD KINEMATICS\n"+"="*70)
    mode=(input("\nMode (N/S) [N]: ").strip().upper() or "N")
    sym=mode.startswith("S")
    
    if sym:
        names=ask_list("  θ1..θ6 symbols: ",6,str)
        q=[sp.Symbol(s) for s in names]
    else:
        unit=(input("Angle unit (deg/rad) [rad]: ").strip().lower() or "rad")
        q=ask_list("  θ1..θ6: ",6,float)
        if unit.startswith("d"): q=[d2r(t) for t in q]
        
        valid,violations=in_limits(robot,q)
        if not valid:
            print("\n  ⚠ Joint limit violations:")
            for j,val,lo,hi in violations:
                print(f"     J{j}: {val:.2f}° (limit: [{lo:.2f}°, {hi:.2f}°])")
    
    Ts,T6=robot.fk_all(q,sym)
    sel=(input("\nPrint frames (all/range) [all]: ").strip().lower() or "all")
    
    if sel.startswith("r"):
        i,j=ask_int("  From: ",1,6),ask_int("  To: ",1,6)
        idx=range(i,j+1)
    else:
        idx=range(1,7)
    
    print("\n"+"="*70+"\nRESULTS\n"+"="*70)
    for k in idx:
        print(f"\n--- Frame {k} ---")
        pprint(f"0T{k}",Ts[k-1],sym=sym)
    
    pprint("0T6 (End-Effector)",T6,sym=sym)

# ============================================================================
# Inverse kinematics
# ============================================================================

# ---------------------------------------------------------------------------
# HARD-CODED SYMBOLIC IK EQUATIONS (no user input)
# ---------------------------------------------------------------------------
def ik_equations_symbolic_hardcoded_kr16(robot: KR16, euler_order: str = "ZYX"):
    """Print SymPy equations used by ik_kr16(), hard-coded symbols (x,y,z,α,β,γ)."""
    x, y, z, alpha, beta, gamma = sp.symbols("x y z alpha beta gamma", real=True)

    R06 = eulerR_s(alpha, beta, gamma, euler_order)
    p06 = sp.Matrix([x, y, z])

    d6 = sp.nsimplify(robot.d[5])
    pwc = sp.simplify(p06 - d6 * R06[:, 2])

    a1 = sp.nsimplify(robot.a[0])
    a2 = sp.Abs(sp.nsimplify(robot.a[1]))
    a3 = sp.nsimplify(robot.a[2])
    d1 = sp.nsimplify(robot.d[0])
    d4 = sp.nsimplify(robot.d[3])

    # θ1 geometry with shoulder offset a1
    rxy = sp.sqrt(pwc[0]**2 + pwc[1]**2)
    phi = sp.atan2(pwc[1], pwc[0])
    s = sp.sqrt(rxy**2 - a1**2)

    th1a = sp.simplify(phi - sp.atan2(a1,  s))
    th1b = sp.simplify(phi - sp.atan2(a1, -s))

    # Transform pwc into frame-1 for planar arm solve
    th1_sym = sp.Symbol("theta1", real=True)
    T01 = dhA(th1_sym, d1, a1, sp.nsimplify(robot.alpha[0]), sym=True)
    p1 = sp.simplify(T01.inv() * sp.Matrix([pwc[0], pwc[1], pwc[2], 1]))
    x1, z1 = sp.simplify(p1[0]), sp.simplify(p1[2])
    r = sp.sqrt(x1**2 + z1**2)  # arm lies in x1–z1 plane for this DH family

    L2 = a2
    L3 = sp.sqrt(a3**2 + d4**2)
    gam = sp.atan2(d4, a3)
    D = sp.simplify((r**2 - L2**2 - L3**2) / (2*L2*L3))
    th3p = sp.Symbol("theta3_prime", real=True)
    th3 = sp.simplify(th3p - gam)

    phi2 = sp.atan2(z1, x1)
    psi2 = sp.atan2(L3*sp.sin(th3p), L2 + L3*sp.cos(th3p))
    th2 = sp.simplify(phi2 - psi2)

    # Orientation: R36 = R03^T R06
    th2_sym = sp.Symbol("theta2", real=True)
    th3_sym = sp.Symbol("theta3", real=True)
    T03 = sp.simplify(dhA(th1_sym, d1, a1, sp.nsimplify(robot.alpha[0]), True) *
                      dhA(th2_sym, sp.nsimplify(robot.d[1]), sp.nsimplify(robot.a[1]), sp.nsimplify(robot.alpha[1]), True) *
                      dhA(th3_sym, sp.nsimplify(robot.d[2]), sp.nsimplify(robot.a[2]), sp.nsimplify(robot.alpha[2]), True))
    R03 = sp.simplify(T03[:3,:3])
    R36 = sp.simplify(R03.T * R06)

    # Wrist extraction matches ik_kr16()
    th5_base = sp.acos(sp.Max(-1, sp.Min(1, R36[2,2])))  # conceptual; numeric clamps
    # Using the same algebraic form as code (θ5 = ±acos(R36[2,2]))
    th5 = sp.Symbol("theta5", real=True)
    s5 = sp.sin(th5)
    th4 = sp.atan2(R36[1,2]/s5, R36[0,2]/s5)
    th6 = sp.atan2(R36[2,1]/s5, -R36[2,0]/s5)

    print("\n" + "="*70)
    print("HARD-CODED SYMBOLIC IK EQUATIONS (KR16) — matches ik_kr16()")
    print("="*70)
    print("\nWrist center: p_wc = p06 - d6*z06   (d6 =", float(robot.d[5]), ")")
    sp.pprint(pwc)

    print("\nθ1 (shoulder offset a1):")
    print("  rxy = sqrt(p_wc_x^2 + p_wc_y^2)")
    print("  φ = atan2(p_wc_y, p_wc_x)")
    print("  s = sqrt(rxy^2 - a1^2)")
    print("  θ1 = φ - atan2(a1, ±s)")
    print("  θ1a =", th1a)
    print("  θ1b =", th1b)

    print("\nTransform to frame-1: p1 = inv(T01(θ1))*[p_wc;1]")
    print("  arm plane is x1–z1, so r = sqrt(x1^2 + z1^2)")
    print("  x1 =", x1)
    print("  z1 =", z1)
    print("  r  =", r)

    print("\nElbow/shoulder (θ2, θ3):")
    print("  D = (r^2 - L2^2 - L3^2)/(2 L2 L3), θ3' = ±acos(D), θ3 = θ3' - γ")
    sp.pprint(D)
    print("  θ2 = atan2(z1,x1) - atan2(L3*sin(θ3'), L2 + L3*cos(θ3'))")
    sp.pprint(th2)

    print("\nWrist (θ4, θ5, θ6):")
    print("  R36 = R03^T R06")
    print("  θ5 = ±acos(R36[2,2])")
    print("  θ4 = atan2(R36[1,2]/sinθ5, R36[0,2]/sinθ5)")
    print("  θ6 = atan2(R36[2,1]/sinθ5, -R36[2,0]/sinθ5)\n")

    return {
        "symbols": (x,y,z,alpha,beta,gamma),
        "pwc": pwc,
        "theta1": (th1a, th1b),
        "D": D,
        "theta2": th2,
        "theta3": th3,
        "R36": R36,
    }


def do_ik(robot):
    print("\n"+"="*70+"\nINVERSE KINEMATICS\n"+"="*70)
    mode=(input("\nMode (N/S) [N]: ").strip().upper() or "N")
    want_sym=mode.startswith("S")
    
    if want_sym:
        ik_equations_symbolic_hardcoded_kr16(robot, euler_order="ZYX")
        return
    
    print("\nInput: 1) 4×4 matrix  2) x,y,z,α,β,γ")
    c=ask_int("Choose: ",1,2)
    
    if c==1:
        T,order=read_T(),"ZYX"
    else:
        T,order=read_pose()
    
    pprint("\nTarget 0T6",T,sym=False)
    
    print("\nSolving...")
    sols=ik_kr16(robot,T)
    
    good=[]
    for q in sols:
        valid,_=in_limits(robot,q)
        if valid and fk_verify(robot,q,T,1e-4):
            good.append(q)
    
    print("\n"+"="*70+"\nRESULTS\n"+"="*70)
    print(f"Raw: {len(sols)} | Valid: {len(good)}")
    
    if not good:
        print("\n  ⚠ No valid solutions!\n     • Unreachable or outside limits")
        return
    
    print("\n--- Solutions (rad) ---")
    for i,q in enumerate(good,1):
        w="  ⚠ WRIST SING" if wrist_sing(q) else ""
        print(f"{i:02d}: [{', '.join(f'{v:+.6f}' for v in q)}]{w}")
    
    print("\n--- Solutions (deg) ---")
    for i,q in enumerate(good,1):
        w="  ⚠ WRIST SING" if wrist_sing(q) else ""
        print(f"{i:02d}: [{', '.join(f'{r2d(v):+7.3f}' for v in q)}]{w}")

def main():
    robot=KR16()
    print("\n"+"="*70+f"\n  {robot.name}\n  FK/IK Calculator (CORRECTED)\n"+"="*70)
    op=(input("\nOperation (F=FK / I=IK) [F]: ").strip().upper() or "F")
    (do_ik if op.startswith("I") else do_fk)(robot)
    print("\n"+"="*70+"\nDone!\n"+"="*70+"\n")

if __name__=="__main__":
    main()