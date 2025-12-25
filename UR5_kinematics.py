#!/usr/bin/env python3
# import math
# import numpy as np
# import sympy as sp
from gui.imports import *
 
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

# Rotation matrices
def Rx(a): ca,sa=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]])
def Ry(b): cb,sb=math.cos(b),math.sin(b); return np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]])
def Rz(c): cc,sc=math.cos(c),math.sin(c); return np.array([[cc,-sc,0],[sc,cc,0],[0,0,1]])

def eulerR(a,b,g,order="ZYX"):
    order=order.upper()
    if order=="ZYX": return Rz(g)@Ry(b)@Rx(a)
    if order=="XYZ": return Rx(a)@Ry(b)@Rz(g)
    raise ValueError("Use ZYX or XYZ")

def Rx_s(a): ca,sa=sp.cos(a),sp.sin(a); return sp.Matrix([[1,0,0],[0,ca,-sa],[0,sa,ca]])
def Ry_s(b): cb,sb=sp.cos(b),sp.sin(b); return sp.Matrix([[cb,0,sb],[0,1,0],[-sb,0,cb]])
def Rz_s(c): cc,sc=sp.cos(c),sp.sin(c); return sp.Matrix([[cc,-sc,0],[sc,cc,0],[0,0,1]])

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
class UR5:
    def __init__(self):
        self.name="UR5"
        self.a=[0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0]
        self.alpha=[PI/2, 0.0, 0.0, PI/2, -PI/2, 0.0]
        self.d=[0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
        self.lim=[(-2*PI, 2*PI)]*6
    
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
def fk_verify(robot,q,Tt,tol=1e-4):
    _,T=robot.fk_all(q,False)
    return allclose(T,Tt,tol)

# Inverse kinematics (UR5 specific)
def ik_ur5(robot,T06):
    a,d,al=robot.a,robot.d,robot.alpha
    R06,p06=T06[:3,:3],T06[:3,3]
    
    d1,d4,d5,d6=d[0],d[3],d[4],d[5]
    a2,a3=a[1],a[2]
    
    sols=[]
    p05=p06-d6*R06[:,2]
    rxy=math.hypot(p05[0],p05[1])
    
    if rxy<1e-9:
        print("  ⚠ Shoulder singularity")
        return sols
    
    c=d4/rxy
    if abs(c)>1.0:
        print(f"  ⚠ Unreachable (d4/rxy={c:.3f})")
        return sols
    
    beta=math.acos(max(-1.0,min(1.0,c)))
    phi=math.atan2(p05[1],p05[0])
    theta1_cands=[wrap(phi+beta+PI/2),wrap(phi-beta+PI/2)]
    
    for th1 in theta1_cands:
        p16=p06[0]*math.sin(th1)-p06[1]*math.cos(th1)
        c5=(p16-d4)/d6
        if abs(c5)>1.0: continue
        
        th5a=math.acos(max(-1.0,min(1.0,c5)))
        
        for th5 in [wrap(th5a),wrap(-th5a)]:
            if abs(math.sin(th5))<1e-8: continue
            
            th6=math.atan2(
                (-R06[0,1]*math.sin(th1)+R06[1,1]*math.cos(th1))/math.sin(th5),
                (R06[0,0]*math.sin(th1)-R06[1,0]*math.cos(th1))/math.sin(th5)
            )
            th6=wrap(th6)
            
            T01=dhA(th1,d1,a[0],al[0],False)
            T45=dhA(th5,d5,a[4],al[4],False)
            T56=dhA(th6,d6,a[5],al[5],False)
            
            T14=np.linalg.inv(T01)@T06@np.linalg.inv(T45@T56)
            p14=T14[:3,3]
            px,py=p14[0],p14[1]
            
            D=(px*px+py*py-a2*a2-a3*a3)/(2*a2*a3)
            if abs(D)>1.0: continue
            D=max(-1.0,min(1.0,D))
            
            for th3 in [math.acos(D),-math.acos(D)]:
                th2=math.atan2(py,px)-math.atan2(a3*math.sin(th3),a2+a3*math.cos(th3))
                th2,th3=wrap(th2),wrap(th3)
                
                qtemp=[th1,th2,th3,0,0,0]
                Ts,_=robot.fk_all(qtemp,sym=False)
                R03=Ts[2][:3,:3]
                R34=R03.T@T14[:3,:3]
                th4=wrap(math.atan2(R34[1,0],R34[0,0]))
                
                sols.append([wrap(th1),wrap(th2),wrap(th3),wrap(th4),wrap(th5),wrap(th6)])
    
    uniq=[]
    for s in sols:
        if not any(all(abs(wrap(s[i]-u[i]))<1e-5 for i in range(6)) for u in uniq):
            uniq.append(s)
    return uniq

# User interaction
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

def print_ik_equations(robot,order="ZYX"):
    x,y,z=sp.symbols("x y z",real=True)
    a,b,g=sp.symbols("alpha beta gamma",real=True)
    R06=eulerR_s(a,b,g,order)
    p06=sp.Matrix([x,y,z])
    pwc=sp.simplify(p06-sp.nsimplify(robot.d[5])*R06[:,2])
    print("\n"+"="*70)
    print("SYMBOLIC IK EQUATIONS")
    print("="*70)
    pprint("R06(α,β,γ)",R06,sym=True)
    pprint("pwc = p06 - d6·z06",pwc,sym=True)
    print("\nUR5: θ1(2) × θ5(2) × θ3(2) → Max 8 solutions")
    print("="*70)

# Forward kinematics
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

# Inverse kinematics
def do_ik(robot):
    print("\n"+"="*70+"\nINVERSE KINEMATICS\n"+"="*70)
    mode=(input("\nMode (N/S) [N]: ").strip().upper() or "N")
    want_sym=mode.startswith("S")
    
    print("\nInput: 1) 4×4 matrix  2) x,y,z,α,β,γ")
    c=ask_int("Choose: ",1,2)
    
    if c==1:
        T,order=read_T(),"ZYX"
    else:
        T,order=read_pose()
        if want_sym and (input("  Show equations? (y/n) [y]: ").strip().lower() or "y").startswith("y"):
            print_ik_equations(robot,order)
    
    pprint("\nTarget 0T6",T,sym=False)
    
    p06=T[:3,3]
    reach=np.linalg.norm(p06)
    max_reach=abs(robot.a[1])+abs(robot.a[2])+abs(robot.d[3])+abs(robot.d[5])
    if reach>max_reach*1.1:
        print(f"\n  ⚠ May be unreachable: {reach:.3f}m (max ~{max_reach:.3f}m)")
    
    print("\nSolving...")
    sols=ik_ur5(robot,T)
    
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

# Main
def main():
    robot=UR5()
    print("\n"+"="*70+f"\n  {robot.name}\n  FK/IK Calculator\n"+"="*70)
    op=(input("\nOperation (F=FK / I=IK) [F]: ").strip().upper() or "F")
    (do_ik if op.startswith("I") else do_fk)(robot)
    print("\n"+"="*70+"\nDone!\n"+"="*70+"\n")

if __name__=="__main__":
    main()