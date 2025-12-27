from imports import *

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2*np.pi) - np.pi

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
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ], dtype=float)
        
def rot_x(a, sym=False):
    c = sp.cos(a) if sym else np.cos(a)
    s = sp.sin(a) if sym else np.sin(a)
    return sp.Matrix([[1,0,0],[0,c,-s],[0,s,c]]) if sym else np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a, sym=False):
    c = sp.cos(a) if sym else np.cos(a)
    s = sp.sin(a) if sym else np.sin(a)
    return sp.Matrix([[c,0,s],[0,1,0],[-s,0,c]]) if sym else np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a, sym=False):
    c = sp.cos(a) if sym else np.cos(a)
    s = sp.sin(a) if sym else np.sin(a)
    return sp.Matrix([[c,-s,0],[s,c,0],[0,0,1]]) if sym else np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy_to_R(alpha, beta, gamma, sym=False):
    # ZYX: Rz(gamma)*Ry(beta)*Rx(alpha)
    return rot_z(gamma, sym) @ rot_y(beta, sym) @ rot_x(alpha, sym)


def in_limits(robot, q):
    msgs = []
    ok = True
    for i,(lo,hi) in enumerate(robot.lim):
        if q[i] < lo-1e-9 or q[i] > hi+1e-9:
            ok = False
            msgs.append(f"θ{i+1} out of limits [{lo:.3f}, {hi:.3f}]")
    return ok, msgs

def allclose(A,B,t=1e-6):
    return np.allclose(A,B,atol=t,rtol=0)
# Base class for robotic manipulators.
# Focuses on DH parameter management and interaction with GUI controls.
class RoboticManipulator:
    # Initialize the manipulator with default DH parameters
    def __init__(self):
        self._dh_params = self._initialize_dh_parameters()
        self.num_joints = len(self._dh_params)
        self.name = "Generic Manipulator"
        self._angle_unit = "radians"  # "radians" or "degrees"
    
    # Override this in child classes to set specific DH parameters.
    # Returns list of dicts with keys: theta, d, a, alpha, variable    
    # Format:
    # {
    #     "theta": float (in radians for computation),
    #     "d": float (meters),
    #     "a": float (meters),
    #     "alpha": float (degrees - will be converted when needed),
    #     "variable": "theta" or "d"
    # }
    def _initialize_dh_parameters(self) -> List[Dict]:
        return []
    

    # Get the DH parameters table.     
    #Returns:List of dicts containing DH parameters
    def get_dh_parameters(self):
        return self._dh_params
    
    # Get specific DH parameter for a joint.
    # Args:
    #     joint_index: Joint number (0-indexed)
    #     param_name: 'theta', 'd', 'a', 'alpha', or 'variable'      
    # Returns:
    #     Parameter value or None if invalid index 
    def get_dh_parameter(self, joint_index, param_name):
        if 0 <= joint_index < self.num_joints:
            return self._dh_params[joint_index].get(param_name)
        return None
    
    
    
    # Get the variable name for a joint (e.g., 'θ₁', 'd₂').      
    # Args:
    #     joint_index: Joint number (0-indexed)  
    # Returns:
    #     String like 'θ₁' or 'd₂'
    def get_joint_variable_name(self, joint_index):
        if 0 <= joint_index < self.num_joints:
            var_type = self._dh_params[joint_index]['variable']
            if var_type == 'theta':
                return f"θ{joint_index + 1}"
            else:
                return f"d{joint_index + 1}"
        return ""
    
    
    
    # Set the angle unit for display and input.
    # Args:
    #     unit: "radians" or "degrees"
    
    def set_angle_unit(self, unit):
        if unit.lower() in ["radians", "degrees"]:
            self._angle_unit = unit.lower()
    
    # Get current angle unit setting
    def get_angle_unit(self):
        return self._angle_unit
    
    
    # Set the value of a joint variable.
    # Stores internally in radians for theta, meters for d.
    
    # Args:
    #     joint_index: Joint number (0-indexed)
    #     value: Joint value
    #     from_degrees: If True and variable is theta, converts from degrees to radians
    def set_joint_value(self, joint_index, value, from_degrees=False):
        if 0 <= joint_index < self.num_joints:
            param = self._dh_params[joint_index]
            
            if param['variable'] == 'theta':
                # Store theta internally in radians
                if from_degrees:
                    param['theta'] = np.deg2rad(value)
                else:
                    param['theta'] = value
            elif param['variable'] == 'd':
                # Store d in meters
                param['d'] = value
 
 
    # Get the current value of a joint variable.
    
    # Args:
    #     joint_index: Joint number (0-indexed)
    #     in_degrees: If True and variable is theta, returns value in degrees
        
    # Returns:
    #     Joint value or None if invalid index 
    def get_joint_value(self, joint_index, in_degrees=False):
        if 0 <= joint_index < self.num_joints:
            param = self._dh_params[joint_index]
            
            if param['variable'] == 'theta':
                val = param['theta']
                return np.rad2deg(val) if in_degrees else val
            elif param['variable'] == 'd':
                return param['d']
        return None
    
    
	# Get all current joint values.    
    # Args:
    #     in_degrees: If True, return theta values in degrees        
    # Returns:
    #     List of joint values
    def get_all_joint_values(self, in_degrees=False):
        values = []
        for i in range(self.num_joints):
            values.append(self.get_joint_value(i, in_degrees))
        return values
        # ==================== FORWARD KINEMATICS ====================
    


    def _T_matrix(self, theta, d, a, alpha):
        """
        Compute DH transformation matrix (numeric)
        Standard DH convention
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ], dtype=float)
    
    def _T_matrix_symbolic(self, theta, d, a, alpha):
        """
        Compute DH transformation matrix (symbolic)
        """
        ct, st = sp.cos(theta), sp.sin(theta)
        ca, sa = sp.cos(alpha), sp.sin(alpha)
        
        return sp.Matrix([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])
        
  
    # ==================== UTILITY FUNCTIONS ====================
    
    def extract_position(self, T):
        """Extract position vector from transformation matrix"""
        return T[:3, 3]
    
    def extract_rotation(self, T):
        """Extract rotation matrix from transformation matrix"""
        return T[:3, :3]
    

        # Convert rotation matrix to Euler angles
        # Args:
        #     R: 3x3 rotation matrix
        #     order: Euler angle convention ('xyz', 'zyx', etc.)
        # Returns:
        #     Tuple of (roll, pitch, yaw) in radians

    def rotation_to_euler(self, R, order='xyz'):
        from scipy.spatial.transform import Rotation as Rot
        r = Rot.from_matrix(R)
        return r.as_euler(order, degrees=False)
    

        # Convert Euler angles to rotation matrix
        # Args:
        #     roll, pitch, yaw: Angles in radians
        #     order: Euler angle convention    
        # Returns:
        #     3x3 rotation matrix
    def euler_to_rotation(self, roll, pitch, yaw, order='xyz'):
        from scipy.spatial.transform import Rotation as Rot
        r = Rot.from_euler(order, [roll, pitch, yaw], degrees=False)
        return r.as_matrix()
   
    
        # Convert pose (position + orientation) to transformation matrix
        # Args:
        #     x, y, z: Position in meters
        #     roll, pitch, yaw: Orientation in radians  
        # Returns:
        #     4x4 transformation matrix
        
    def pose_to_matrix(self, x, y, z, roll, pitch, yaw):
        R = self.euler_to_rotation(roll, pitch, yaw)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T
    
    def do_ik_symbolic(self):
        x = sp.Symbol('x', real=True)
        y = sp.Symbol('y', real=True)
        z = sp.Symbol('z', real=True)
        alpha = sp.Symbol('α', real=True)
        beta = sp.Symbol('β', real=True)
        gamma = sp.Symbol('γ', real=True)
        
        R = rpy_to_R(alpha, beta, gamma, sym=True)
        T = sp.Matrix([[R[0,0],R[0,1],R[0,2],x],
                       [R[1,0],R[1,1],R[1,2],y],
                       [R[2,0],R[2,1],R[2,2],z],
                       [0,0,0,1]])
        return T
    
    def fk_all(self, q, sym=False):
        individual_Ts = []  # Individual: 1→2, 2→3, etc.
        cumulative_Ts = []  # Cumulative: 0→1, 0→2, 0→3, etc.
        T = sp.eye(4) if sym else np.eye(4)
        
        q_symbols = None
        if sym:
            q_symbols = [sp.Symbol(f'θ{i+1}', real=True) for i in range(6)]
            q = q_symbols  # Use symbols instead of numeric values
    
        for i in range(6):
            A = dhA(q[i], self.d[i], self.a[i], self.alpha[i], sym=sym)
            individual_Ts.append(A)  # Store individual transformation
            T = T @ A
            cumulative_Ts.append(T) 

        if sym:
            return  individual_Ts, cumulative_Ts, T, q_symbols
        else:
            return individual_Ts, cumulative_Ts, T
    
    





class UR5(RoboticManipulator):
    """Universal Robots UR5 manipulator"""
    
    def __init__(self):
        super().__init__()
        self.name = "UR5"
    
    #   UR5 DH parameters (Modified DH convention)
    def _initialize_dh_parameters(self) :
        self.d1 = 0.089159
        self.a2= -0.425
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
        self.lim = [(-2*pi, 2*pi)]*6
        self.a = [0.0, self.a2, self.a3, 0.0, 0.0, 0.0]
        self.alpha = [ pi/2, 0.0, 0.0, pi/2, -pi/2, 0.0]
        self.d = [self.d1, 0.0, 0.0, self.d4, self.d5, self.d6]

        return [
            {"a": 0, "alpha": pi / 2,  "d": self.d1, "theta": 0.0, "variable": "theta"},
            {"a": self.a2, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a3, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": self.d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2, "d": self.d5, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": 0,   "d": self.d6, "theta": 0.0, "variable": "theta"},
        ]  
         


    # -------------------- CLOSED FORM IK (CONSISTENT WITH FK ABOVE) --------------------
    def ik_ur5_closed_form(self, T06: np.ndarray):
        d1, a2, a3, d4, d5, d6 = self.d1, self.a2, self.a3, self.d4, self.d5, self.d6

        R06 = T06[:3, :3]
        p06 = T06[:3, 3]
        px, py, pz = float(p06[0]), float(p06[1]), float(p06[2])

        sols = []

        # wrist center (p05)
        p05 = p06 - d6 * R06[:, 2]
        rxy = np.hypot(p05[0], p05[1])
        if rxy < 1e-12:
            return []

        # θ1 (two solutions)
        c = d4 / rxy
        if abs(c) > 1.0 + 1e-9:
            return []
        c = np.clip(c, -1.0, 1.0) 
        phi = np.arccos(c)
        psi = np.arctan2(p05[1], p05[0])
        q1_candidates = [wrap_angle(psi + phi + pi/2), wrap_angle(psi - phi + pi/2)]

        for q1 in q1_candidates:
            s1, c1 = np.sin(q1), np.cos(q1)

            # θ5 (two solutions)
            c5 = (px*s1 - py*c1 - d4) / d6
            if abs(c5) > 1.0 + 1e-9:
                continue
            c5 = np.clip(c5, -1.0, 1.0) 
            q5a = np.arccos(c5)
            q5_candidates = [wrap_angle(q5a), wrap_angle(-q5a)]

            for q5 in q5_candidates:
                s5 = np.sin(q5)
                if abs(s5) < 1e-10:
                    continue

                # θ6
                q6 = np.arctan2( 
                    (-R06[0,1]*s1 + R06[1,1]*c1) / s5,
                    (R06[0,0]*s1 - R06[1,0]*c1) / s5
                )
                q6 = wrap_angle(q6)

                # Compute T14 = inv(T01)*T06*inv(T45*T56)
                T01 = dhA(q1, d1, 0.0, pi/2, sym=False)
                T45 = dhA(q5, d5, 0.0, -pi/2, sym=False)
                T56 = dhA(q6, d6, 0.0, 0.0, sym=False)

                T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)

                # θ2, θ3 from planar in frame1: use x,z (NOT x,y)
                p14 = T14[:3, 3]
                x, y = float(p14[0]), float(p14[1])

                D = (x*x + y*y - a2*a2 - a3*a3) / (2*a2*a3)
                if abs(D) > 1.0 + 1e-9:
                    continue
                D = np.clip(D, -1.0, 1.0)

                for q3 in [np.arccos(D), -np.arccos(D)]:
                    q2 = np.arctan2(y, x) - np.arctan2(a3*np.sin(q3), a2 + a3*np.cos(q3))
                    q2, q3 = wrap_angle(q2), wrap_angle(q3)

                    # θ4 from rotation
                    qtemp = [q1, q2, q3, 0.0, 0.0, 0.0]
                    _, cumulative_Ts, _ = self.fk_all(qtemp, sym=False)
                    R03 = cumulative_Ts[2][:3, :3]
                    R04 = (T01 @ T14)[:3, :3]
                    R34 = R03.T @ R04
                    q4 = wrap_angle(np.arctan2(R34[1,0], R34[0,0]))

                    candidate = [wrap_angle(q1), wrap_angle(q2), wrap_angle(q3), wrap_angle(q4), wrap_angle(q5), wrap_angle(q6)]

                    # Validate by FK to guarantee consistency
                    _, _, Tchk = self.fk_all(candidate, sym=False)
                    if np.allclose(Tchk, T06, atol=1e-6, rtol=0):
                        sols.append(candidate)

        # Remove duplicates (wrap-aware)
        uniq = []
        for s in sols:
            if not any(sum((wrap_angle(si-ui))**2 for si,ui in zip(s,u)) < 1e-10 for u in uniq):
                uniq.append(s)
        return uniq


# -------------------- SYMBOLIC IK (Equations only) --------------------

 












#ABB IRB1600 industrial manipulator
class ABB_IRB_1600(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "ABB_IRB_1600"
    
        # ABB IRB1600 DH parameters
    def _initialize_dh_parameters(self):
        self.a1 = 0.448
        self.a2 = 1.066
        self.a3 = 0.114
        self.d1 = 0.72
        self.d4 = 1.002
        self.d6 = 0.25
        self.a=[self.a1, self.a2, self.a3, 0.0, 0.0, 0.0]
        self.alpha=[-pi/2, 0.0, -pi/2, pi/2, -pi/2, 0.0]
        self.d=[self.d1, 0.0, 0.0, self.d4, 0.0, self.d6]
        self.lim = [
            (np.deg2rad(-180), np.deg2rad(180)),
            (np.deg2rad(-120), np.deg2rad(120)),
            (np.deg2rad(-200), np.deg2rad(200)),
            (np.deg2rad(-180), np.deg2rad(180)),
            (np.deg2rad(-120), np.deg2rad(120)),
            (np.deg2rad(-360), np.deg2rad(360))
        ]
        return [
            {"a": self.a1, "alpha": -pi / 2,  "d": self.d1, "theta": 0.0, "variable": "theta"},
            {"a": self.a2, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a3, "alpha": -pi / 2 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2, "d": self.d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": 0, "d": self.d6, "theta": 0.0, "variable": "theta"},
        ]
        
    
    # def wrist_sing(q): 
    #     return abs(math.sin(q[4]))<1e-6
    
    # def shoulder_sing(pwc): 
    #     return math.hypot(pwc[0],pwc[1])<1e-3
    
    # def elbow_sing(r,s,L2,L3):
    #     reach=math.sqrt(r*r+s*s)
    #     return reach>L2+L3-1e-3 or reach<abs(L2-L3)+1e-3

    # def fk_verify(robot,q,Tt,tol=1e-4):
    #     _,T=robot.fk_all(q,False)
    #     return allclose(T,Tt,tol)

    # Inverse kinematics (FIXED, COMPLETE BRANCHES)
    def ik_irb1600_closed_form(self, T06: np.ndarray):
        """
        Closed-form IK for the 6R ABB IRB1600-style arm (spherical wrist), consistent with THIS file's DH.

        Returns up to 8 solutions: (theta1 2 branches) × (theta3 2 branches) × (theta5 flip 2 branches)
        """
        a1, a2, a3, d1, d4, d6 = self.a1, self.a2, self.a3, self.d1, self.d4, self.d6
        R06, p06 = T06[:3, :3], T06[:3, 3]

        # Wrist center (origin of frame-5). Here a6=0, alpha6=0 => tool offset is along z6 only.
        pwc = p06 - d6 * R06[:, 2]

        # Arm geometry for first 3 joints
        L2 = abs(a2)
        L3 = np.hypot(a3, d4)
        gamma = np.arctan2(d4, a3)

        solutions = []

        # Two base branches for theta1
        th1_base = np.arctan2(pwc[1], pwc[0])
        th1_candidates = [wrap_angle(th1_base), wrap_angle(th1_base + pi)]

        for th1 in th1_candidates:
            T01 = dhA(th1, d1, a1, -pi/2, sym=False)
            p1 = np.linalg.inv(T01) @ np.array([pwc[0], pwc[1], pwc[2], 1.0])

            # IMPORTANT: for THIS DH, joints 2&3 triangle is in (x1,y1) plane
            x, y = float(p1[0]), float(p1[1])

             # Law of cosines for elbow
            D = (x * x + y * y - L2 * L2 - L3 * L3) / (2 * L2 * L3)
            if abs(D) > 1.0 + 1e-9:
                continue
            D = np.clip(D, -1.0, 1.0)

            # 2 elbow branches
            for th3p in [np.arccos(D), -np.arccos(D)]:
                th3 = th3p - gamma
                th2 = np.arctan2(y, x) - np.arctan2(L3 * np.sin(th3p), L2 + L3 * np.cos(th3p))

                # Compute R03 then R36
                qtmp = [th1, th2, th3, 0.0, 0.0, 0.0]
                _, cumulative_Ts, _ = self.fk_all(qtmp, sym=False)
                R03 = cumulative_Ts[2][:3, :3]
                R36 = R03.T @ R06

                # Wrist extraction 
                th5 = np.arctan2(np.hypot(R36[2, 0], R36[2, 1]), R36[2, 2])

                # 2 wrist flip branches
                for sgn in [1, -1]:
                    th5c = sgn * th5
                    if abs(np.sin(th5c)) < 1e-10:
                        # Wrist singularity: choose a valid pair (theta4, theta6)
                        th4 = np.arctan2(R36[1, 0], R36[0, 0])
                        th6 = 0.0
                    else:
                        if sgn > 0:
                            th4 = np.arctan2(R36[1, 2], R36[0, 2])
                            th6 = np.arctan2(-R36[2, 1], R36[2, 0])
                        else:
                            th4 = np.arctan2(-R36[1, 2], -R36[0, 2])
                            th6 = np.arctan2(R36[2, 1], -R36[2, 0])

                    # DH frame convention in this file requires +pi on theta4
                    th4 = wrap_angle(th4 + pi)
                    
                    candidate = [
                        wrap_angle(th1),
                        wrap_angle(th2),
                        wrap_angle(th3),
                        wrap_angle(th4),
                        wrap_angle(th5c),
                        wrap_angle(th6)
                    ]
                    
                    _, _, Tchk = self.fk_all(candidate, sym=False)
                    if allclose(Tchk, T06, t=1e-6):
                        solutions.append(candidate)


        # # Remove near-duplicates
        # uniq = []
        # for s in solutions:
        #     if not any(all(abs(wrap(s[i] - u[i])) < 1e-6 for i in range(6)) for u in uniq):
        #         uniq.append(s)
        # return uniq
        # Remove duplicates
        unique_solutions = []
        for s in solutions:
            if not any(sum((wrap_angle(si-ui))**2 for si, ui in zip(s, u)) < 1e-10 
                      for u in unique_solutions):
                unique_solutions.append(s)
        
        return unique_solutions
    
    







class KUKA_KR16(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "KUKA_KR16"
    
        # KUKA KR16 DH parameters
    def _initialize_dh_parameters(self):
        self.a1 = 0.26
        self.a2 = 0.68
        self.a3 = 0.035
        self.d1 = 0.675
        self.d4 = 0.67
        self.d6 = 0.115
        
        self.a = [self.a1, self.a2, self.a3, 0.0, 0.0, 0.0]
        self.alpha = [pi/2, 0.0, pi/2, -pi/2, pi/2, 0.0]
        self.d = [self.d1, 0.0, 0.0, self.d4, 0.0, self.d6]
        
        self.lim=[(np.deg2rad(-185), np.deg2rad(185)), (np.deg2rad(-35),np.deg2rad(158)), (np.deg2rad(-120), np.deg2rad(158)),
                  (np.deg2rad(-350), np.deg2rad(350)), (np.deg2rad(-130),np.deg2rad(130)), (np.deg2rad(-350), np.deg2rad(350))]
    
        return [
            {"a": self.a1, "alpha": pi / 2,  "d": self.d1, "theta": 0.0, "variable": "theta"},
            {"a": self.a2, "alpha":  0.0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a3, "alpha": pi / 2 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0.0, "alpha":  -pi / 2, "d": self.d4, "theta": 0.0, "variable": "theta"},
            {"a": 0.0, "alpha": pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0.0, "alpha":  0.0, "d": self.d6, "theta": 0.0, "variable": "theta"},
        ]
        
        """
        Forward kinematics - SAME STRUCTURE AS UR5
        
        Args:
            q: Joint angles [θ1, θ2, θ3, θ4, θ5, θ6] (radians)
            sym: Boolean, if True use symbolic computation
            
        Returns:
            individual_Ts: List of individual link transformations
            cumulative_Ts: List of cumulative transformations from base
            T: Final end-effector transformation
            q_symbols: (only if sym=True) symbolic joint variables
        """
        


    # Analytical IK for KUKA KR16 with spherical wrist.
    # Returns unique solutions (up to 8) that pass FK verification.
    # DH pattern assumed (as in this script):
    #   a=[a1,a2,a3,0,0,0], d=[d1,0,0,d4,0,d6]
    #   alpha=[±pi/2,0,±pi/2,∓pi/2,±pi/2,0]
    def ik_kuka_kr16_closed_form(self, T06: np.ndarray):
        a,d,alpha = self.a, self.d, self.alpha
        R06, p06 = T06[:3,:3], T06[:3,3]

        # Wrist center (frame 5 origin)
        a1,a2,a3 = a[0], a[1], a[2]
        d1, d4, d6 = self.d1, self.d4, self.d6
        
        pwc = p06 - d6 * R06[:,2]


        # q1 candidates (account for shoulder offset a1)
        xw,yw,zw = pwc.tolist()
        r_xy = np.hypot(xw, yw)
        
        th1_list = []
        if abs(a1) < 1e-10:
            th1_list = [np.arctan2(yw, xw)]
        else:
            if r_xy < abs(a1) - 1e-9:
                return []
            phi = np.arctan2(yw, xw)
            s = np.sqrt(max(0.0, r_xy*r_xy - a1*a1))
            # Two geometric solutions
            th1_list = [phi - np.arctan2(a1,  s),
                        phi - np.arctan2(a1, -s)]

        sols = []
        # Precompute helpers
        L2 = abs(a2)
        L3 = np.hypot(a3, d4)
        gamma = np.arctan2(d4, a3)  # elbow offset between a3 and d4

        for th1 in th1_list:
            # Transform wrist center into frame1
            T01 = dhA(th1, d1, a1, alpha[0], False)
            p1 = np.linalg.inv(T01) @ np.array([xw,yw,zw,1.0])
            x1, y1, z1 = p1[0], p1[1], p1[2]

            # For this DH family, the 2R "arm" lies in the x1-z1 plane
            r = np.hypot(x1, z1)

            # Reachability (with tolerance)
            if r > (L2 + L3) + 1e-8 or r < abs(L2 - L3) - 1e-8:
                continue

            cos_th3p = (r*r - L2*L2 - L3*L3) / (2*L2*L3)
            cos_th3p = np.clip(cos_th3p, -1.0, 1.0)

            for th3p in [np.arccos(cos_th3p), -np.arccos(cos_th3p)]:
                th3 = th3p - gamma

                phi = np.arctan2(z1, x1)
                psi = np.arctan2(L3*np.sin(th3p), L2 + L3*np.cos(th3p))
                th2 = phi - psi

                # Wrist orientation
                qtemp = [th1, th2, th3, 0.0, 0.0, 0.0]
                _, cumulative_Ts, _ = self.fk_all(qtmp, sym=False)
                R03 = cumulative_Ts[2][:3, :3]
                R36 = R03.T @ R06

                # th5 from R36[2,2], th4/th6 from remaining terms
                c5 = np.clip(R36[2, 2], -1.0, 1.0)
                th5_base = np.arccos(c5)
                for th5 in [th5_base, -th5_base]:
                    s5 = np.sin(th5)
                    if abs(s5) < 1e-10:
                        th4 = wrap_angle(np.arctan2(-R36[1,0], R36[0,0]))
                        th6 = 0.0
                    else:
                        th4 = wrap_angle(np.arctan2(R36[1,2]/s5, R36[0,2]/s5))
                        th6 = wrap_angle(np.arctan2(R36[2,1]/s5, -R36[2,0]/s5))

                    candidate = [
                        wrap_angle(th1),
                        wrap_angle(th2),
                        wrap_angle(th3),
                        wrap_angle(th4),
                        wrap_angle(th5),
                        wrap_angle(th6)
                    ]

                    # FK verification
                    _, _, Tchk = self.fk_all(candidate, sym=False)
                    if allclose(Tchk, T06, t=1e-5):
                        sols.append(candidate)
                        
        unique_solutions = []
        for s in sols:
            # Wrap-aware duplicate check
            is_duplicate = any(
                np.allclose(np.array(s), np.array(u), atol=1e-4, rtol=0)
                for u in unique_solutions
            )
            if not is_duplicate:
                unique_solutions.append(s)
        
        return unique_solutions




# ==================== FACTORY FUNCTION ====================

    # Factory function to create manipulator instances.
    # Args:
    #     name: Manipulator name ("UR5", "ABB IRB1600", "ABB IRB2600")  
    # Returns:
    #     Instance of the appropriate manipulator class
def create_manipulator(name):

    manipulators = {
        "UR5": UR5,
        "ABB_IRB_1600": ABB_IRB_1600,
        "KUKA_KR16": KUKA_KR16,
    }
    
    manipulator_class = manipulators.get(name)
    if manipulator_class:
        return manipulator_class()
    else:
        raise ValueError(f"Unknown manipulator: {name}")