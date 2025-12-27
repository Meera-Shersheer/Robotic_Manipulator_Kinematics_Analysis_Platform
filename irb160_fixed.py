#!/usr/bin/env python3
import math
import numpy as np
import sympy as sp

PI = math.pi

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
        print("\n  Enter symbolic task-space variables:")
        var_names=ask_list("  x y z Î± Î² Î³ symbols: ",6,str)
        x_sym, y_sym, z_sym, a_sym, b_sym, g_sym = [sp.Symbol(v, real=True) for v in var_names]
        
        order=(input("  Euler order (ZYX/XYZ) [ZYX]: ").strip().upper() or "ZYX")
        
        R06=eulerR_s(a_sym,b_sym,g_sym,order)
        p06=sp.Matrix([x_sym,y_sym,z_sym])
        
        d6_sym=sp.nsimplify(robot.d[5])
        pwc=p06 - d6_sym*R06[:,2]
        
        a2_sym,a3_sym=sp.nsimplify(robot.a[1]),sp.nsimplify(robot.a[2])
        d4_sym=sp.nsimplify(robot.d[3])
        L2_sym=sp.Abs(a2_sym)
        L3_sym=sp.sqrt(a3_sym**2 + d4_sym**2)
        gamma_sym=sp.atan2(d4_sym,a3_sym)
        
        r_sym,s_sym=sp.symbols('r s',real=True)
        th3p_sym=sp.Symbol('theta_3prime',real=True)
        
        print("\n"+"="*70)
        print("SYMBOLIC INVERSE KINEMATICS EQUATIONS")
        print("="*70)
        
        print("\n" + "â”€"*70)
        print("BLOCK 1: Rotation Matrix")
        print("â”€"*70)
        pprint("Râ‚€â‚†(Î±,Î²,Î³)",R06,sym=True)
        
        print("\n" + "â”€"*70)
        print("BLOCK 2: Wrist Center")
        print("â”€"*70)
        print(f"dâ‚† = {robot.d[5]}")
        pprint("p_wc = pâ‚€â‚† - dâ‚†Â·zâ‚€â‚†",pwc,sym=True)
        
        print("\n" + "â”€"*70)
        print("BLOCK 3: Joint 1 (Base Rotation)")
        print("â”€"*70)
        print("Î¸â‚ = atan2(p_wc,y, p_wc,x)")
        print("Alternative: Î¸â‚' = Î¸â‚ + Ï€")
        print("â†’ 2 solutions")
        
        print("\n" + "â”€"*70)
        print("BLOCK 4: Transform to Frame 1")
        print("â”€"*70)
        print("pâ‚ = â°Tâ‚â»Â¹(Î¸â‚) Â· [p_wc,x, p_wc,y, p_wc,z, 1]áµ€")
        print("r = pâ‚,x")
        print("s = pâ‚,y")
        print("(So rÂ²+sÂ² = pâ‚,xÂ² + pâ‚,yÂ²)")
        
        print("\n" + "â”€"*70)
        print("BLOCK 5: Link Geometry")
        print("â”€"*70)
        print("Lâ‚‚ = |aâ‚‚|")
        sp.pprint(L2_sym)
        print("\nLâ‚ƒ = âˆš(aâ‚ƒÂ² + dâ‚„Â²)")
        sp.pprint(L3_sym)
        print("\nÎ³ = atan2(dâ‚„, aâ‚ƒ)")
        sp.pprint(gamma_sym)
        
        print("\n" + "â”€"*70)
        print("BLOCK 6: Joint 3 (Elbow) - Law of Cosines")
        print("â”€"*70)
        D_sym=(r_sym**2 + s_sym**2 - L2_sym**2 - L3_sym**2)/(2*L2_sym*L3_sym)
        print("D = (rÂ² + sÂ² - Lâ‚‚Â² - Lâ‚ƒÂ²)/(2Lâ‚‚Lâ‚ƒ)")
        sp.pprint(D_sym)
        print("\nÎ¸â‚ƒ' = Â±acos(D)")
        print("Î¸â‚ƒ = Î¸â‚ƒ' - Î³")
        print("â†’ 2 solutions (elbow up/down)")
        
        print("\n" + "â”€"*70)
        print("BLOCK 7: Joint 2 (Shoulder)")
        print("â”€"*70)
        print("Î¸â‚‚ = atan2(s, r) - atan2(Lâ‚ƒÂ·sin(Î¸â‚ƒ'), Lâ‚‚ + Lâ‚ƒÂ·cos(Î¸â‚ƒ'))")
        th2_formula=sp.atan2(s_sym,r_sym) - sp.atan2(L3_sym*sp.sin(th3p_sym), L2_sym + L3_sym*sp.cos(th3p_sym))
        sp.pprint(th2_formula)
        
        print("\n" + "â”€"*70)
        print("BLOCK 8: Wrist Orientation (Joints 4, 5, 6)")
        print("â”€"*70)
        print("Compute: Râ‚€â‚ƒ then Râ‚ƒâ‚† = Râ‚€â‚ƒáµ€ Â· Râ‚€â‚†")
        print("Î¸â‚… = atan2(âˆš(Râ‚ƒâ‚†[2,0]Â² + Râ‚ƒâ‚†[2,1]Â²), Râ‚ƒâ‚†[2,2])")
        print("Then extract Î¸â‚„, Î¸â‚† and add +Ï€ offset on Î¸â‚„ for this DH convention.")
        print("\nAlternative: Î¸â‚…' = -Î¸â‚…  â†’ 2 wrist-flip solutions")
        
        print("\n" + "â”€"*70)
        print("BLOCK 9: Solution Structure")
        print("â”€"*70)
        print("Total branches: 2 (Î¸1) Ã— 2 (Î¸3) Ã— 2 (Î¸5 flip) = 8 solutions max")
        
        print("="*70)
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
