#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import sympy as sp

PI = math.pi
PI_SYM = sp.pi

# Utilities
def d2r(x): return x*PI/180.0
def r2d(x): return x*180.0/PI
def wrap(x): return (x+PI)%(2*PI)-PI
def allclose(A,B,t=1e-6): return np.allclose(A,B,atol=t,rtol=0)

def ask_int(msg, lo=None, hi=None):
    while True:
        try:
            v=int(input(msg).strip())
            if lo and v<lo: print(f"  âš   >= {lo}"); continue
            if hi and v>hi: print(f"  âš   <= {hi}"); continue
            return v
        except: print("  âš   Invalid integer")

def ask_list(msg,n,cast=float):
    while True:
        s=input(msg).strip().replace(","," ")
        p=s.split()
        if len(p)!=n: print(f"  âš   Need {n} values"); continue
        try: return [cast(x) for x in p]
        except: print("  âš   Invalid format")

def pprint(name, M, sym=False):
    print(f"\n{name} =")
    if sym: sp.pprint(M)
    else:
        with np.printoptions(precision=6, suppress=True, linewidth=100): print(M)

# Rotation matrices
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

# DH transformation
def dhA(th,d,a,al,sym=False):
    if sym:
        ct,st,ca,sa=sp.cos(th),sp.sin(th),sp.cos(al),sp.sin(al)
        return sp.Matrix([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
    ct,st,ca,sa=math.cos(th),math.sin(th),math.cos(al),math.sin(al)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0.0,sa,ca,d],[0.0,0.0,0.0,1.0]])

# Robot class
class IRB1600:
    def __init__(self):
        self.name="ABB IRB1600"
        self.a=[0.448, 1.066, 0.114, 0.0, 0.0, 0.0]
        self.alpha=[-PI/2, 0.0, -PI/2, PI/2, -PI/2, 0.0]
        self.d=[0.72, 0.0, 0.0, 1.002, 0.0, 0.25]
        self.lim=[(d2r(-180),d2r(180)),(d2r(-120),d2r(120)),(d2r(-200),d2r(200)),
                  (d2r(-180),d2r(180)),(d2r(-120),d2r(120)),(d2r(-360),d2r(360))]
    
    def fk_all(self,q,sym=False):
        T=sp.eye(4) if sym else np.eye(4)
        Ts=[]
        for i in range(6):
            T=sp.simplify(T*dhA(q[i],self.d[i],self.a[i],self.alpha[i],True)) if sym else T@dhA(q[i],self.d[i],self.a[i],self.alpha[i],False)
            Ts.append(T if sym else T.copy())
        return Ts, Ts[-1]

# Validation
def in_limits(robot,q):
    violations=[]
    for i,(lo,hi) in enumerate(robot.lim):
        if q[i]<lo-1e-9 or q[i]>hi+1e-9:
            violations.append((i+1,r2d(q[i]),r2d(lo),r2d(hi)))
    return len(violations)==0, violations

def wrist_sing(q): return abs(math.sin(q[4]))<1e-6
def shoulder_sing(pwc): return math.hypot(pwc[0],pwc[1])<1e-3
def elbow_sing(r,s,L2,L3):
    reach=math.sqrt(r*r+s*s)
    return reach>L2+L3-1e-3 or reach<abs(L2-L3)+1e-3

def fk_verify(robot,q,Tt,tol=1e-4):
    _,T=robot.fk_all(q,False)
    return allclose(T,Tt,tol)

# Inverse kinematics (FIXED, COMPLETE BRANCHES)
def ik_spherical(robot: IRB1600, T06: np.ndarray, tol: float = 1e-9):
    """
    Closed-form IK for the 6R ABB IRB1600-style arm (spherical wrist), consistent with THIS file's DH.

    Returns up to 8 solutions: (theta1 2 branches) × (theta3 2 branches) × (theta5 flip 2 branches)
    """
    a, d, alpha = robot.a, robot.d, robot.alpha
    R06, p06 = T06[:3, :3], T06[:3, 3]

    # Wrist center (origin of frame-5). Here a6=0, alpha6=0 => tool offset is along z6 only.
    pwc = p06 - d[5] * R06[:, 2]

    # Arm geometry for first 3 joints
    a2, a3, d4 = a[1], a[2], d[3]
    L2 = abs(a2)
    L3 = math.sqrt(a3 * a3 + d4 * d4)
    gamma = math.atan2(d4, a3)

    solutions = []

    # Two base branches for theta1
    th1_base = math.atan2(pwc[1], pwc[0])
    th1_candidates = [wrap(th1_base), wrap(th1_base + math.pi)]

    for th1 in th1_candidates:
        T01 = dhA(th1, d[0], a[0], alpha[0], sym=False)
        p1 = np.linalg.inv(T01) @ np.array([pwc[0], pwc[1], pwc[2], 1.0])

        # IMPORTANT: for THIS DH, joints 2&3 triangle is in (x1,y1) plane
        x, y = float(p1[0]), float(p1[1])

        D = (x * x + y * y - L2 * L2 - L3 * L3) / (2 * L2 * L3)
        if abs(D) > 1.0 + tol:
            continue
        D = max(-1.0, min(1.0, D))

        # 2 elbow branches
        for th3p in [math.acos(D), -math.acos(D)]:
            th3 = th3p - gamma
            th2 = math.atan2(y, x) - math.atan2(L3 * math.sin(th3p), L2 + L3 * math.cos(th3p))

            # Compute R03 then R36
            qtmp = [th1, th2, th3, 0.0, 0.0, 0.0]
            Ts, _ = robot.fk_all(qtmp, sym=False)
            R03 = Ts[2][:3, :3]
            R36 = R03.T @ R06

            # Wrist extraction consistent with DH wrist
            th5 = math.atan2(math.hypot(R36[2, 0], R36[2, 1]), R36[2, 2])

            # 2 wrist flip branches
            for sgn in [1, -1]:
                th5c = sgn * th5
                if abs(math.sin(th5c)) < 1e-10:
                    # Wrist singularity: choose a valid pair (theta4, theta6)
                    th4 = math.atan2(R36[1, 0], R36[0, 0])
                    th6 = 0.0
                else:
                    if sgn > 0:
                        th4 = math.atan2(R36[1, 2], R36[0, 2])
                        th6 = math.atan2(-R36[2, 1], R36[2, 0])
                    else:
                        th4 = math.atan2(-R36[1, 2], -R36[0, 2])
                        th6 = math.atan2(R36[2, 1], -R36[2, 0])

                # DH frame convention in this file requires +pi on theta4
                th4 = wrap(th4 + math.pi)

                solutions.append([wrap(th1), wrap(th2), wrap(th3), wrap(th4), wrap(th5c), wrap(th6)])

    # Remove near-duplicates
    uniq = []
    for s in solutions:
        if not any(all(abs(wrap(s[i] - u[i])) < 1e-6 for i in range(6)) for u in uniq):
            uniq.append(s)
    return uniq


# ---------------------------------------------------------------------------
# HARD-CODED SYMBOLIC IK EQUATIONS (no user input)
# ---------------------------------------------------------------------------
def ik_equations_symbolic_hardcoded(robot: IRB1600, euler_order: str = "ZYX"):
    """Print SymPy equations used by ik_spherical(), hard-coded symbols (x,y,z,α,β,γ).

    This does NOT solve numerically; it only displays the analytic equations/branches that
    match this file's DH/FK conventions (same as ik_spherical()).
    """
    x, y, z, alpha, beta, gamma = sp.symbols("x y z alpha beta gamma", real=True)

    # Target transform 0T6
    R06 = eulerR_s(alpha, beta, gamma, euler_order)
    p06 = sp.Matrix([x, y, z])

    # Wrist center (frame-5 origin): p_wc = p06 - d6 * z06
    d6 = sp.nsimplify(robot.d[5])
    pwc = sp.simplify(p06 - d6 * R06[:, 2])

    # Arm geometry for joints (2,3) with this DH
    a2 = sp.Abs(sp.nsimplify(robot.a[1]))
    a3 = sp.nsimplify(robot.a[2])
    d4 = sp.nsimplify(robot.d[3])

    L2 = a2
    L3 = sp.sqrt(a3**2 + d4**2)
    gam = sp.atan2(d4, a3)  # "gamma" elbow offset (use 'gam' to avoid name clash)

    # θ1 (two branches)
    th1 = sp.atan2(pwc[1], pwc[0])
    th1b = sp.simplify(th1 + sp.pi)

    # Transform wrist center into frame-1: p1 = inv(T01)*[pwc;1]
    th1_sym = sp.Symbol("theta1", real=True)
    T01 = dhA(th1_sym, sp.nsimplify(robot.d[0]), sp.nsimplify(robot.a[0]), sp.nsimplify(robot.alpha[0]), sym=True)
    p1 = sp.simplify(T01.inv() * sp.Matrix([pwc[0], pwc[1], pwc[2], 1]))

    # IMPORTANT for THIS DH: triangle is in (x1,y1) plane
    r = sp.simplify(p1[0])
    s = sp.simplify(p1[1])

    # Law of cosines for θ3'
    D = sp.simplify((r**2 + s**2 - L2**2 - L3**2) / (2*L2*L3))
    th3p = sp.Symbol("theta3_prime", real=True)  # θ3' = ±acos(D)
    th3 = sp.simplify(th3p - gam)

    # θ2
    th2 = sp.simplify(sp.atan2(s, r) - sp.atan2(L3*sp.sin(th3p), L2 + L3*sp.cos(th3p)))

    # Orientation: compute R03 then R36 = R03^T R06
    th2_sym = sp.Symbol("theta2", real=True)
    th3_sym = sp.Symbol("theta3", real=True)
    q1,q2,q3 = th1_sym, th2_sym, th3_sym
    T03 = sp.simplify(dhA(q1, sp.nsimplify(robot.d[0]), sp.nsimplify(robot.a[0]), sp.nsimplify(robot.alpha[0]), True) *
                      dhA(q2, sp.nsimplify(robot.d[1]), sp.nsimplify(robot.a[1]), sp.nsimplify(robot.alpha[1]), True) *
                      dhA(q3, sp.nsimplify(robot.d[2]), sp.nsimplify(robot.a[2]), sp.nsimplify(robot.alpha[2]), True))
    R03 = sp.simplify(T03[:3,:3])
    R36 = sp.simplify(R03.T * R06)

    # Wrist extraction (same pattern as ik_spherical())
    th5 = sp.atan2(sp.sqrt(R36[2,0]**2 + R36[2,1]**2), R36[2,2])

    # For the "non-flip" branch (θ5 positive):
    th4_nf = sp.atan2(R36[1,2], R36[0,2])
    th6_nf = sp.atan2(-R36[2,1], R36[2,0])

    # For the "flip" branch (θ5 negative):
    th4_f  = sp.atan2(-R36[1,2], -R36[0,2])
    th6_f  = sp.atan2(R36[2,1], -R36[2,0])

    # This DH convention needs +π on θ4 then wrap
    th4_nf = sp.simplify(th4_nf + sp.pi)
    th4_f  = sp.simplify(th4_f  + sp.pi)

    print("\n" + "="*70)
    print("HARD-CODED SYMBOLIC IK EQUATIONS (IRB1600) — matches ik_spherical()")
    print("="*70)

    print("\nTarget rotation R06(α,β,γ) with Euler order:", euler_order)
    sp.pprint(R06)

    print("\nWrist center: p_wc = p06 - d6*z06   (d6 =", float(robot.d[5]), ")")
    sp.pprint(pwc)

    print("\nθ1 branches:")
    print("  θ1  = atan2(p_wc_y, p_wc_x)")
    print("  θ1' = θ1 + π")
    print("  θ1 =", th1)
    print("  θ1'=", th1b)

    print("\nTransform to frame-1 (use θ1): p1 = inv(T01(θ1))*[p_wc;1]")
    print("  r = p1_x ,  s = p1_y   (triangle is in x1–y1 plane for this DH)")
    print("  r =", r)
    print("  s =", s)

    print("\nLink geometry:")
    print("  L2 = |a2| =", L2)
    print("  L3 = sqrt(a3^2 + d4^2) =", L3)
    print("  γ  = atan2(d4, a3) =", gam)

    print("\nElbow (θ3') and θ3:")
    print("  D = (r^2+s^2 - L2^2 - L3^2) / (2 L2 L3)")
    sp.pprint(D)
    print("  θ3' = ±acos(D)")
    print("  θ3  = θ3' - γ")
    print("  θ3  =", th3)

    print("\nShoulder θ2:")
    sp.pprint(th2)

    print("\nOrientation:")
    print("  R36 = R03^T * R06")
    print("  θ5 = atan2( sqrt(R36[2,0]^2 + R36[2,1]^2), R36[2,2] )")
    print("  θ4,θ6 depend on wrist flip:")
    print("    non-flip: θ4 = atan2(R36[1,2], R36[0,2]) + π ; θ6 = atan2(-R36[2,1], R36[2,0])")
    print("    flip:     θ4 = atan2(-R36[1,2], -R36[0,2]) + π ; θ6 = atan2(R36[2,1], -R36[2,0])")
    print("\n(Use wrap() numerically; symbolic mode shows raw atan2/acos forms.)\n")

    return {
        "symbols": (x,y,z,alpha,beta,gamma),
        "pwc": pwc,
        "theta1": (th1, th1b),
        "D": D,
        "theta2": th2,
        "theta3": th3,
        "R36": R36,
        "theta4_nonflip": th4_nf,
        "theta6_nonflip": th6_nf,
        "theta5": th5,
        "theta4_flip": th4_f,
        "theta6_flip": th6_f,
    }


# User interaction
def read_T():
    print("\n  Enter 16 numbers (row-major):")
    v=ask_list("  T: ",16,float)
    return np.array(v,float).reshape(4,4)

def read_pose():
    print("\n  Enter position and orientation:")
    v=ask_list("  x y z Î± Î² Î³: ",6,float)
    x,y,z,al,be,ga=v
    unit=(input("  Angle unit (deg/rad) [deg]: ").strip().lower() or "deg")
    if unit.startswith("d"): al,be,ga=d2r(al),d2r(be),d2r(ga)
    order=(input("  Euler order (ZYX/XYZ) [ZYX]: ").strip().upper() or "ZYX")
    R=eulerR(al,be,ga,order)
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[x,y,z]
    return T,order

# Forward kinematics
def do_fk(robot):
    print("\n"+"="*70+"\nFORWARD KINEMATICS\n"+"="*70)
    mode=(input("\nMode (N/S) [N]: ").strip().upper() or "N")
    sym=mode.startswith("S")
    
    if sym:
        names=ask_list("  Î¸1..Î¸6 symbols: ",6,str)
        q=[sp.Symbol(s) for s in names]
    else:
        unit=(input("Angle unit (deg/rad) [rad]: ").strip().lower() or "rad")
        q=ask_list("  Î¸1..Î¸6: ",6,float)
        if unit.startswith("d"): q=[d2r(t) for t in q]
        
        valid,violations=in_limits(robot,q)
        if not valid:
            print("\n  âš   Joint limit violations:")
            for j,val,lo,hi in violations:
                print(f"     J{j}: {val:.2f}Â° (limit: [{lo:.2f}Â°, {hi:.2f}Â°])")
    
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
    
    if 6 not in idx:
        pprint("0T6 (End-Effector)",T6,sym=sym)

# Inverse kinematics
def do_ik(robot):
    print("\n"+"="*70+"\nINVERSE KINEMATICS\n"+"="*70)
    mode=(input("\nMode (N/S) [N]: ").strip().upper() or "N")
    want_sym=mode.startswith("S")
    if want_sym:
        # Hard-coded symbolic equations (no prompts)
        ik_equations_symbolic_hardcoded(robot, euler_order="ZYX")
    return

    
    print("\nInput: 1) 4Ã—4 matrix  2) x,y,z,Î±,Î²,Î³")
    c=ask_int("Choose: ",1,2)
    
    if c==1:
        T,order=read_T(),"ZYX"
    else:
        T,order=read_pose()
    
    pprint("\nTarget 0T6",T,sym=False)
    
    p06=T[:3,3]
    reach=np.linalg.norm(p06)
    max_reach=sum(abs(a) for a in robot.a)+sum(abs(d) for d in robot.d)
    if reach>max_reach:
        print(f"\n  âš   May be unreachable: {reach:.3f}m (max ~{max_reach:.3f}m)")
    
    print("\nSolving...")
    sols=ik_spherical(robot,T)
    
    good=[]
    for q in sols:
        valid,_=in_limits(robot,q)
        if valid and fk_verify(robot,q,T,1e-4):
            good.append(q)
    
    print("\n"+"="*70+"\nRESULTS\n"+"="*70)
    print(f"Raw: {len(sols)} | Valid: {len(good)}")
    
    if not good:
        print("\n  âš   No valid solutions!\n     â€¢ Unreachable or outside limits")
        return
    
    print("\n--- Solutions (rad) ---")
    for i,q in enumerate(good,1):
        w="  âš   WRIST SING" if wrist_sing(q) else ""
        print(f"{i:02d}: [{', '.join(f'{v:+.6f}' for v in q)}]{w}")
    
    print("\n--- Solutions (deg) ---")
    for i,q in enumerate(good,1):
        w="  âš   WRIST SING" if wrist_sing(q) else ""
        print(f"{i:02d}: [{', '.join(f'{r2d(v):+7.3f}' for v in q)}]{w}")

def main():
    robot=IRB1600()
    print("\n"+"="*70+f"\n  {robot.name}\n  FK/IK Calculator\n"+"="*70)
    op=(input("\nOperation (F=FK / I=IK) [F]: ").strip().upper() or "F")
    (do_ik if op.startswith("I") else do_fk)(robot)
    print("\n"+"="*70+"\nDone!\n"+"="*70+"\n")

if __name__=="__main__":
    main()
