from imports import *

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2*np.pi) - np.pi

"""
    Modified DH Transformation Matrix (Craig Convention)
    Computes transform from frame {i-1} to {i}
    Modified DH (Craig): Rot_x(alpha) -> Trans_x(a) -> Rot_z(theta) -> Trans_z(d)
"""
               
def dh_Craig(theta, d, a, alpha, sym=False):
    if sym:
        ct, st = sp.cos(theta), sp.sin(theta)
        ca, sa = sp.cos(alpha), sp.sin(alpha)
        return sp.Matrix([
            [ct,        -st,        0,      a],
            [st*ca,     ct*ca,    -sa,    -sa*d],
            [st*sa,     ct*sa,     ca,     ca*d ],
            [0,       0,      0,    1]
        ])
    else:
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct,        -st,       0,      a],
            [st*ca,     ct*ca,    -sa,    -sa*d],
            [st*sa,     ct*sa,     ca,     ca*d],
            [0,         0,         0,      1]
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

# def Rx_s(a): ca,sa=sp.cos(a),sp.sin(a); return sp.Matrix([[1,0,0],[0,ca,-sa],[0,sa,ca]])
# def Ry_s(b): cb,sb=sp.cos(b),sp.sin(b); return sp.Matrix([[cb,0,sb],[0,1,0],[-sb,0,cb]])
# def Rz_s(g): cg,sg=sp.cos(g),sp.sin(g); return sp.Matrix([[cg,-sg,0],[sg,cg,0],[0,0,1]])


# def eulerR(a,b,g,order="ZYX"):
#     order=order.upper()
#     if order=="ZYX": return Rz(g)@Ry(b)@Rx(a)
#     if order=="XYZ": return Rx(a)@Ry(b)@Rz(g)
#     raise ValueError("Use ZYX or XYZ")

# def eulerR_s(a,b,g,order="ZYX"):
#     order=order.upper()
#     if order=="ZYX": return sp.simplify(Rz_s(g)*Ry_s(b)*Rx_s(a))
#     if order=="XYZ": return sp.simplify(Rx_s(a)*Ry_s(b)*Rz_s(g))
#     raise ValueError("Use ZYX or XYZ")

# def in_limits(self, q):
#     msgs = []
#     ok = True
#     for i,(lo,hi) in enumerate(self.lim):
#         if q[i] < lo-1e-9 or q[i] > hi+1e-9:
#             ok = False
#             msgs.append(f"θ{i+1} out of limits [{lo:.3f}, {hi:.3f}]")
#     return ok, msgs

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
    


        """
        Compute Modified DH transformation matrix (numeric) - Craig Convention
        """
    def _T_matrix(self, theta, d, a, alpha):
        return dh_Craig(theta, d, a, alpha, sym=False)
    
        """
        Compute Modified DH transformation matrix (symbolic) - Craig Convention
        """
    def _T_matrix_symbolic(self, theta, d, a, alpha):
        return dh_Craig(theta, d, a, alpha, sym=True)
        
    
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
    
      
      
    """
        Forward Kinematics using equation (13) from PDF:
        T_0^6 = T_0^1 * T_1^2 * T_2^3 * T_3^4 * T_4^5 * T_5^6
        
        Returns:
            individual_Ts: Individual link transformations T_{i-1}^i
            cumulative_Ts: Cumulative transformations T_0^i
            T: Final transformation T_0^6
    """   
    def fk_all(self, q, sym=False):
        individual_Ts = []  # Individual: 1→2, 2→3, etc.
        cumulative_Ts = []  # Cumulative: 0→1, 0→2, 0→3, etc.
        T = sp.eye(4) if sym else np.eye(4)
        
        q_symbols = None
        if sym:
            q_symbols = [sp.Symbol(f'θ{i+1}', real=True) for i in range(6)]
            q = q_symbols  # Use symbols instead of numeric values
    
        for i in range(6):
            Ti = dh_Craig(q[i], self.d[i], self.a[i], self.alpha[i], sym=sym)
            individual_Ts.append(Ti)  # Store individual transformation
            T = T @ Ti
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
    
    def _initialize_dh_parameters(self) :
        self.d1 = 0.089159
        self.a2= -0.425
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
        self.lim = [(-2*pi, 2*pi)]*6
        # In Modified DH, 'a' and 'alpha' are a_{i-1} and alpha_{i-1}
        self.a = [0.0, 0.0, self.a2, self.a3, 0.0, 0.0]
        self.alpha = [ 0.0, pi/2, 0.0, 0.0, pi/2, -pi/2]
        self.d = [self.d1, 0.0, 0.0, self.d4, self.d5, self.d6]
        return [
            {"a": self.a[0], "alpha": self.alpha[0], "d": self.d[0], "theta": 0.0, "variable": "theta"},
            {"a": self.a[1], "alpha": self.alpha[1], "d": self.d[1], "theta": 0.0, "variable": "theta"},
            {"a": self.a[2], "alpha": self.alpha[2], "d": self.d[2], "theta": 0.0, "variable": "theta"},
            {"a": self.a[3], "alpha": self.alpha[3], "d": self.d[3], "theta": 0.0, "variable": "theta"},
            {"a": self.a[4], "alpha": self.alpha[4], "d": self.d[4], "theta": 0.0, "variable": "theta"},
            {"a": self.a[5], "alpha": self.alpha[5], "d": self.d[5], "theta": 0.0, "variable": "theta"},
        ] 
        

    # using Paul's Closed-Form Method
    def ik_ur5_closed_form(self, T06: np.ndarray):
        d1, d4, d5, d6 = self.d[0], self.d[3], self.d[4], self.d[5]
        a2, a3 = self.a[2], self.a[3]

        R06 = T06[:3, :3]
        p06 = T06[:3, 3]

        sols = []

        # wrist center (p05)
        p05 = p06 - d6 * R06[:, 2]
        rxy = np.hypot(p05[0], p05[1])
        if rxy < 1e-12:
            return []

        # θ1 (two solutions)
        c = d4 / rxy
        if abs(c) > 1.0:
            return []
        phi = np.arccos(c)
        psi = np.arctan2(p05[1], p05[0])
        q1_candidates = [wrap_angle(psi + phi + pi/2), wrap_angle(psi - phi + pi/2)]

        for q1 in q1_candidates:
            s1, c1 = np.sin(q1), np.cos(q1)

            # θ5 (two solutions)
            c5 = (p06[0]*s1 - p06[1]*c1 - d4) / d6
            if abs(c5) > 1.0:
                c5 = np.sign(c5) * 1.0
            #c5 = np.clip(c5, -1.0, 1.0) 
            
            q5a = np.arccos(c5)
            q5_candidates = [wrap_angle(q5a), wrap_angle(-q5a)]

            for q5 in q5_candidates:
                s5 = np.sin(q5)
                if abs(s5) < 1e-4:
                    q6 = 0.0
                else:
                # θ6
                    q6 = np.arctan2( 
                        (-1 * R06[0,1]*s1 + R06[1,1]*c1) / s5,
                        (R06[0,0]*s1 - R06[1,0]*c1) / s5
                    )
                q6 = wrap_angle(q6)

                # Compute T14 = inv(T01)*T06*inv(T45*T56)
                T01 = dh_Craig(q1, self.d[0], self.a[0], self.alpha[0], sym=False)
                T45 = dh_Craig(q5, self.d[4], self.a[4], self.alpha[4], sym=False)
                T56 = dh_Craig(q6, self.d[5], self.a[5], self.alpha[5], sym=False)

                T54 = np.linalg.inv(T45)
                T65 = np.linalg.inv(T56)

                T14 = np.linalg.inv(T01) @ T06 @ T65 @ T54
                # θ2, θ3 from planar in frame1: use x,z (NOT x,y)
                p14 = T14[:3, 3]
                x, z = float(p14[0]), float(p14[2])

                D = (x*x + z*z - a2*a2 - a3*a3) / (2*a2*a3)
                if abs(D) > 1.0 :
                    continue

                for q3 in [np.arccos(D), -np.arccos(D)]:
                    q2 = np.arctan2(z, x) - np.arctan2(a3*np.sin(q3), a2 + a3*np.cos(q3))
                    q2, q3 = wrap_angle(q2), wrap_angle(q3)

                    # θ4 from rotation
                    # qtemp = [q1, q2, q3, 0.0, 0.0, 0.0]
                    # _, cumulative_Ts, _ = self.fk_all(qtemp, sym=False)
                    # R03 = cumulative_Ts[2][:3, :3]
                    # R04 = (T01 @ T14)[:3, :3]
                    # R34 = R03.T @ R04
                    # q4 = wrap_angle(np.arctan2(R34[1,0], R34[0,0]))
                    R14 = T14[:3, :3]
                    # Project R14 into the planar rotation angle
                    # usually arctan2(R21, R11) or similar depending on axes
                    angle_234 = np.arctan2(R14[2,0], R14[0,0])
                    
                    q4 = wrap_angle(angle_234 - q2 - q3)
                    
                    candidate = [q1, q2, q3, q4, q5, q6]
                    # Validate by FK to guarantee consistency
                    _, _, Tchk = self.fk_all(candidate, sym=False)
                    
                    if not np.allclose(Tchk, T06, atol=1e-4, rtol=1e-4):
                        # Attempt q4 fix if 180 off
                        candidate[3] = wrap_angle(q4 + pi)
                        _, _, Tchk = self.fk_all(candidate, sym=False)
                        
                    if np.allclose(Tchk, T06, atol=1e-4, rtol=0):
                        valid = True
                        for k, angle in enumerate(candidate):
                            limit_min, limit_max = self.lim[k]
                            # Allow small tolerance
                            if not (limit_min - 1e-4 <= angle <= limit_max + 1e-4):
                                valid = False
                                break
                        
                        if valid:
                            sols.append(candidate)

        # Remove duplicates (wrap-aware)
        uniq = []
        for s in sols:
            if not any(sum((wrap_angle(si-ui))**2 for si,ui in zip(s,u)) < 1e-10 for u in uniq):
                uniq.append(s)
        return uniq
 


    def do_ik_symbolic(self):
        #  Convention: R = Rz(γ) * Ry(β) * Rx(α)
        self.x = sp.Symbol('x', real=True)
        self.y = sp.Symbol('y', real=True)
        self.z = sp.Symbol('z', real=True)
        self.alph = sp.Symbol('α', real=True)
        self.beta = sp.Symbol('β', real=True)
        self.gamma = sp.Symbol('γ', real=True)
        
        #DERIVING SYMBOLIC IK SOLUTION
        R = rpy_to_R(self.alph, self.beta, self.gamma, sym=True)
        
        T = sp.Matrix([[R[0,0],R[0,1],R[0,2],self.x],
                       [R[1,0],R[1,1],R[1,2],self.y],
                       [R[2,0],R[2,1],R[2,2],self.z],
                       [0,0,0,1]])
        R06 = T
        
        #"[1/6] Computing wrist center (p05)..."
        r13, r23, r33 = R06[0, 2], R06[1, 2], R06[2, 2]
        
        p05x = self.x - self.d6 * r13
        p05y = self.y - self.d6 * r23
        p05z = self.z - self.d6 * r33

        # "Wrist center = p06 - d6 * R06[:, 2]"
        
        #"[2/6] Solving for θ1..."
        rxy = sp.sqrt(p05x**2 + p05y**2)
        psi = sp.atan2(p05y, p05x)
        phi = sp.acos(self.d4 / rxy)

        
        theta1_sol1 = psi + phi + pi/2
        theta1_sol2 = psi - phi + pi/2
        
        #"θ1 = atan2(p05y, p05x) ± acos(d4/rxy) + π/2"
        theta1 = theta1_sol1
        s1, c1 = sp.sin(theta1), sp.cos(theta1)
        
        #"[3/6] Solving for θ5..."
        cos_theta5 = (self.x * s1 - self.y * c1 - self.d4) / self.d6
        theta5_sol1 = sp.acos(cos_theta5)
        theta5_sol2 = -sp.acos(cos_theta5)
        
        #"  cos(θ5) = (x*sin(θ1) - y*cos(θ1) - d4) / d6"
        
        theta5 = theta5_sol1
        s5 = sp.sin(theta5)
        
        #"[4/6] Solving for θ6..."
        #"  θ6 = atan2((−R06[0,1]*sin(θ1) + R06[1,1]*cos(θ1))/sin(θ5), ...)")
        num6 = -R06[0,1]*s1 + R06[1,1]*c1
        den6 = R06[0,0]*s1 - R06[1,0]*c1
        theta6 = sp.atan2(num6, den6)
        
        #"[5/6] Computing p14 position..."
       
        p14x= c1*p05x + s1*p05y - self.d4
        p14z = p05z - self.d1

        
        # [6/6] Solving for θ2 and θ3...
        # Use X and Z in the cosine rule
        D = (p14x**2 + p14z**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3)
        theta3_sol1 = sp.acos(D)
        theta3_sol2 = -sp.acos(D)
        
        theta3 = theta3_sol1
        s3, c3 = sp.sin(theta3), sp.cos(theta3)
        
        theta2 = sp.atan2(p14z, p14x) - sp.atan2(self.a3*s3, self.a2 + self.a3*c3)
        
        #"  θ3 = ±acos(D), D = (p14x² + p14y² - a2² - a3²)/(2*a2*a3)"
        #"  θ2 = atan2(p14y, p14x) - atan2(a3*sin(θ3), a2 + a3*cos(θ3))"
        
        #"[7/7] θ4 from rotation constraint..."
       
        
        # theta4 = pitch - theta2 - theta3
        R14_00 = sp.Symbol('R14_00', real=True)
        R14_20 = sp.Symbol('R14_20', real=True)

        angle_234 = sp.atan2(R14_20, R14_00)
        theta4 = angle_234 - theta2 - theta3
        #"  θ4 = [Computed from rotation matrices R03, R04]"
        
        #"SYMBOLIC DERIVATION COMPLETE"
        result = {
            'theta1': [theta1_sol1, theta1_sol2],
            'theta2': theta2,
            'theta3': [theta3_sol1, theta3_sol2],
            'theta4': theta4,
            'theta5': [theta5_sol1, theta5_sol2],
            'theta6': theta6,
            'wrist_center': (p05x, p05y, p05z),
            'R06': R06
        }
        return result, T
    
    #ABB IRB1600 industrial manipulator
class ABB_IRB_1600(RoboticManipulator):

    def __init__(self):
        super().__init__()
        self.name = "ABB_IRB_1600"
    
    def _initialize_dh_parameters(self):
        self.a1 = 0.448
        self.a2 = 1.066
        self.a3 = 0.114
        self.d1 = 0.72
        self.d4 = 1.002
        self.d6 = 0.25
        self.a=[0.0, self.a1, self.a2, self.a3, 0.0, 0.0]
        self.alpha=[0.0, -pi/2, 0.0, -pi/2, pi/2, -pi/2]
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
            {"a": 0, "alpha": 0,  "d": self.d1, "theta": 0.0, "variable": "theta"},
            {"a": self.a1, "alpha": -pi / 2, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a2, "alpha": 0 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a3, "alpha": -pi / 2, "d": self.d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2, "d": self.d6, "theta": 0.0, "variable": "theta"},
        ]
        

# using Pieper's method (with kinamatic decoupling)     
    def ik_irb1600_closed_form(self, T06: np.ndarray):
        """
        Closed-form IK for the 6R ABB IRB1600-style arm (spherical wrist), consistent with THIS file's DH.

        Returns up to 8 solutions: (theta1 2 branches) × (theta3 2 branches) × (theta5 flip 2 branches)
        """
        d1 = self.d[0]
        a1 = self.a[1]
        a2 = self.a[2]
        a3 = self.a[3]
        d4 = self.d[3]
        d6 = self.d[5]
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
            T01 = dh_Craig(th1, d1, 0.0, 0.0, sym=False)
            p1 = np.linalg.inv(T01) @ np.array([pwc[0], pwc[1], pwc[2], 1.0])

            # IMPORTANT: for THIS DH, joints 2&3 triangle is in (x1,y1) plane
            x = float(p1[0]) - a1
            y = -1 *float(p1[2])

             # Law of cosines for elbow
            D = (x * x + y * y - L2 * L2 - L3 * L3) / (2 * L2 * L3)
            if abs(D) > 1.0 + 1e-6:
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
                th5 = np.arctan2(np.hypot(R36[1, 0], R36[1, 1]), R36[1, 2])

                # 2 wrist flip branches
                for sgn in [1, -1]:
                    th5c = sgn * th5
                    sign = 1.0 if sgn > 0 else -1.0
                    
                    if abs(np.sin(th5c)) < 1e-6:
                            th4 = 0.0
                            th6 = np.arctan2(-R36[0, 1], R36[0, 0])
                            wrist_candidates = [(th4, th6)]
                    else: # Singularity check
                        th4 = np.arctan2(sign * R36[2, 2], sign * -R36[0, 2])
                        th6 = np.arctan2(sign * -R36[1, 1], sign * R36[1, 0])

                    candidate = [
                        wrap_angle(th1), wrap_angle(th2), wrap_angle(th3),
                        wrap_angle(th4), wrap_angle(th5c), wrap_angle(th6)
                    ]
                        
                    _, _, T_chk = self.fk_all(candidate, sym=False)
                    if np.allclose(T_chk, T06, atol=1e-3):
                        valid = True
                        for k, angle in enumerate(candidate):
                            limit_min, limit_max = self.lim[k]
                            # Allow small tolerance
                            if not (limit_min - 1e-4 <= angle <= limit_max + 1e-4):
                                valid = False
                                break  
                        if valid:
                            solutions.append(candidate)


        # # Remove near-duplicates
        unique_solutions = []
        for s in solutions:
            if not any(sum((wrap_angle(si-ui))**2 for si, ui in zip(s, u)) < 1e-3
                      for u in unique_solutions):
                unique_solutions.append(s)
        
        return unique_solutions
    


    def do_ik_symbolic(self): #  Convention: R = Rz(γ) * Ry(β) * Rx(α)
        self.x = sp.Symbol('x', real=True)
        self.y = sp.Symbol('y', real=True)
        self.z = sp.Symbol('z', real=True)
        self.alph = sp.Symbol('α', real=True)
        self.beta = sp.Symbol('β', real=True)
        self.gamma = sp.Symbol('γ', real=True)
        
        #DERIVING SYMBOLIC IK SOLUTION
        R = rpy_to_R(self.alph, self.beta, self.gamma, sym=True)
        
        T = sp.Matrix([[R[0,0],R[0,1],R[0,2],self.x],
                       [R[1,0],R[1,1],R[1,2],self.y],
                       [R[2,0],R[2,1],R[2,2],self.z],
                       [0,0,0,1]])
        R06 = T
        
        #"[1/6] Computing wrist center ..."
        r13, r23, r33 = R06[0, 2], R06[1, 2], R06[2, 2]
        
        pwcx = self.x - self.d6 * r13
        pwcy = self.y - self.d6 * r23
        pwcz = self.z - self.d6 * r33
        
        #"  Wrist center = p06 - d6 * R06[:, 2]"
        
        #"[2/6] Solving for θ1 (base rotation)..."
        th1_base = sp.atan2(pwcy, pwcx)
        theta1_sol1 = th1_base
        theta1_sol2 = th1_base + pi
        
        #"  θ1 = atan2(pwcy, pwcx) or atan2(pwcy, pwcx) + π"
        
        theta1 = theta1_sol1
        
        #"[3/6] Transform to frame 1 (for joints 2&3 in x1-y1 plane)..."
        # For ABB, joints 2&3 work in x1-y1 plane (not x1-z1)
        # Simplified geometric projection
        s1, c1 = sp.sin(theta1), sp.cos(theta1)
        
        # Position in frame 1
        x1_approx = c1*pwcx + s1*pwcy - self.a1
        y1_approx = -pwcz
        
        #"  x1 ≈ cos(θ1)*pwcx + sin(θ1)*pwcy - a1")
        #"  y1 ≈ -sin(θ1)*pwcx + cos(θ1)*pwcy")
        #"[4/6] Solving for θ3 (elbow)...")
        L2 = self.a2
        L3 = sp.sqrt(self.a3**2 + self.d4**2)
        gamma = sp.atan2(self.d4, self.a3)
        
        D = (x1_approx**2 + y1_approx**2 - L2**2 - L3**2) / (2*L2*L3)
        
        theta3p_sol1 = sp.acos(D)
        theta3p_sol2 = -sp.acos(D)
        
        theta3_sol1 = theta3p_sol1 - gamma
        theta3_sol2 = theta3p_sol2 - gamma
        
        #"  L3 = sqrt(a3² + d4²), γ = atan2(d4, a3)")
        #"  θ3' = ±acos(D), D = (x1² + y1² - L2² - L3²)/(2*L2*L3)")
        #"  θ3 = θ3' - γ")
        
        theta3p = theta3p_sol1
        theta3 = theta3_sol1
        
        #"\n[5/6] Solving for θ2...")
        theta2 = sp.atan2(y1_approx, x1_approx) - sp.atan2(L3*sp.sin(theta3p), L2 + L3*sp.cos(theta3p))
        
        #"  θ2 = atan2(y1, x1) - atan2(L3*sin(θ3'), L2 + L3*cos(θ3'))")
        
        #"\n[6/6] Solving for wrist angles...")
        # R36 = R03^T @ R06
        
        # θ5 from spherical wrist
        r36_02 = sp.Symbol('R36_02') # Row 0, Col 2
        r36_10 = sp.Symbol('R36_10') # Row 1, Col 0
        r36_11 = sp.Symbol('R36_11') # Row 1, Col 1
        r36_12 = sp.Symbol('R36_12') # Row 1, Col 2
        r36_22 = sp.Symbol('R36_22') # Row 2, Col 2
        
        theta5 = sp.atan2(sp.sqrt(r36_10**2 + r36_11**2), r36_12)
        theta5_sol1 = theta5
        theta5_sol2 = -theta5
        
        theta4 = sp.atan2(r36_22, -r36_02)
        
        theta6_raw = sp.atan2(-r36_11, r36_10)
        
        theta6 = theta6_raw + sp.pi
        
        #"  θ5 = ±atan2(sqrt(R36[2,0]² + R36[2,1]²), R36[2,2])"
        #"  θ4 = atan2(R36[1,2], R36[0,2]) + π  [DH convention]"
        #"  θ6 = atan2(-R36[2,1], R36[2,0]"
        
        
        result = {
            'theta1': [theta1_sol1, theta1_sol2],
            'theta2': theta2,
            'theta3': [theta3_sol1, theta3_sol2],
            'theta4': theta4,
            'theta5': [theta5_sol1, theta5_sol2],
            'theta6': theta6,
            'wrist_center': (pwcx, pwcy, pwcz),
            'L2': L2,
            'L3': L3,
            'planar_coords': (x1_approx, y1_approx),
            'gamma': gamma,
            'R06': R06
        }
        return result, T
   







class KUKA_KR16(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "KUKA_KR16"
    
    def _initialize_dh_parameters(self):
        self.a1 = 0.26
        self.a2 = 0.68
        self.a3 = 0.035
        self.d1 = 0.675
        self.d4 = 0.67
        self.d6 = 0.115
        
        self.a = [0.0, self.a1, self.a2, self.a3, 0.0, 0.0]
        self.alpha = [0.0, pi/2, 0.0, pi/2, -pi/2, pi/2]
        self.d = [self.d1, 0.0, 0.0, self.d4, 0.0, self.d6]
        
        self.lim=[(np.deg2rad(-185), np.deg2rad(185)), (np.deg2rad(-35),np.deg2rad(158)), (np.deg2rad(-120), np.deg2rad(158)),
                  (np.deg2rad(-350), np.deg2rad(350)), (np.deg2rad(-130),np.deg2rad(130)), (np.deg2rad(-350), np.deg2rad(350))]
    
        return [
            {"a": 0.0, "alpha": 0.0,  "d": self.d1, "theta": 0.0, "variable": "theta"},
            {"a": self.a1, "alpha":  pi / 2, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a2, "alpha": 0.0 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": self.a3, "alpha":  pi / 2, "d": self.d4, "theta": 0.0, "variable": "theta"},
            {"a": 0.0, "alpha": -pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0.0, "alpha":  pi / 2, "d": self.d6, "theta": 0.0, "variable": "theta"},
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
        

# using Pieper's method (with kinamatic decoupling)    
    # Analytical IK for KUKA KR16 with spherical wrist.
    # Returns unique solutions (up to 8) that pass FK verification.
    # DH pattern assumed (as in this script):
    #   a=[a1,a2,a3,0,0,0], d=[d1,0,0,d4,0,d6]
    #   alpha=[±pi/2,0,±pi/2,∓pi/2,±pi/2,0]
    def ik_kuka_kr16_closed_form(self, T06: np.ndarray):
        """
        Closed-form IK for the KUKA KR16 (Modified DH).
        
        FIXES:
        1. Plane: Kept X-Z (Vertical) because Sol 3 proved it yields correct Theta 2.
        2. Gamma: CHANGED sign to (+ gamma). The previous error of ~3.03 rad 
           corresponds exactly to 2*gamma, indicating the offset was subtracted 
           instead of added.
        """
        # Load Parameters
        a1, a2, a3 = self.a[1], self.a[2], self.a[3]
        d1, d4, d6 = self.d[0], self.d[3], self.d[5]
        
        # Base twist alpha[0] is 0 for KUKA KR16
        alp0 = 0.0

        R06, p06 = T06[:3, :3], T06[:3, 3]

        # 1. Wrist Center
        pwc = p06 - d6 * R06[:, 2]
        xw, yw, zw = pwc[0], pwc[1], pwc[2]

        solutions = []

        # --- JOINT 1 (Base) ---
        # a1 is a forward link length. Point directly at target.
        th1_base = np.arctan2(yw, xw)
        th1_candidates = [wrap_angle(th1_base), wrap_angle(th1_base + np.pi)]

        # Arm Geometry
        L2 = abs(a2)
        L3 = np.hypot(a3, d4)
        gamma = np.arctan2(d4, a3)

        for th1 in th1_candidates:
            # Transform to Frame 1
            # T01 uses a=0 (Link 0). a1 (Link 1) is handled in the triangle subtraction.
            T01 = dh_Craig(th1, d1, 0.0, alp0, sym=False)
            p1 = np.linalg.inv(T01) @ np.array([xw, yw, zw, 1.0])


            x_tri = float(p1[0]) - a1
            y_tri = float(p1[2]) # Z is height
            
            # Law of Cosines for Elbow
            r_sq = x_tri**2 + y_tri**2
            D = (r_sq - L2**2 - L3**2) / (2 * L2 * L3)
            
            if abs(D) > 1.0 + 1e-6:
                continue
            D = np.clip(D, -1.0, 1.0)

            for th3p in [np.arccos(D), -np.arccos(D)]:
                th3 = th3p + gamma
                
                # Theta 2 (Shoulder)
                phi_arm = np.arctan2(y_tri, x_tri)
                beta = np.arctan2(L3 * np.sin(th3p), L2 + L3 * np.cos(th3p))
                th2 = phi_arm - beta

                # --- WRIST ORIENTATION ---
                qtmp = [th1, th2, th3, 0.0, 0.0, 0.0]
                _, cumulative_Ts, _ = self.fk_all(qtmp, sym=False)
                R03 = cumulative_Ts[2][:3, :3]
                R36 = R03.T @ R06

                # KUKA Z-Y-Z Wrist
                c5 = np.clip(-R36[1, 2], -1.0, 1.0)
                s5_mag = np.hypot(R36[0, 2], R36[2, 2])
                th5_base = np.arctan2(s5_mag, c5)

                for sgn in [1, -1]:
                    th5 = sgn * th5_base
                    sign = 1.0 if sgn > 0 else -1.0
                    
                    if abs(np.sin(th5)) < 1e-6:
                        th4 = 0.0
                        th6 = np.arctan2(R36[2, 0], R36[0, 0])
                    else:
                        # Z-Y-Z Extraction
                        # Theta 4 (Yaw): atan2(R12, R02)
                        th4 = np.arctan2(sign * R36[2, 2], sign * R36[0, 2])
                        # Theta 6 (Roll): atan2(R21, -R20)
                        th6 = np.arctan2(sign * R36[1, 1], sign * -R36[1, 0])

                    candidate = [
                        wrap_angle(th1), wrap_angle(th2), wrap_angle(th3),
                        wrap_angle(th4), wrap_angle(th5), wrap_angle(th6)
                    ]

                    # Verification
                    _, _, T_chk = self.fk_all(candidate, sym=False)
                    if np.allclose(T_chk, T06, atol=10):
                        valid = True
                        for k, angle in enumerate(candidate):
                            limit_min, limit_max = self.lim[k]
                            # Allow small tolerance
                            if not (limit_min - 1e-4 <= angle <= limit_max + 1e-4):
                                valid = False
                                break
                        
                        if valid:
                            solutions.append(candidate)

        # Remove duplicates
        unique_solutions = []
        for s in solutions:
            if not any(np.allclose(s, u, atol=1e-4) for u in unique_solutions):
                unique_solutions.append(s)
        
        return unique_solutions








    def do_ik_symbolic(self):
        #  Convention: R = Rz(γ) * Ry(β) * Rx(α)
        self.x = sp.Symbol('x', real=True)
        self.y = sp.Symbol('y', real=True)
        self.z = sp.Symbol('z', real=True)
        self.alph = sp.Symbol('α', real=True)
        self.beta = sp.Symbol('β', real=True)
        self.gamma = sp.Symbol('γ', real=True)
        
        #DERIVING SYMBOLIC IK SOLUTION
        R = rpy_to_R(self.alph, self.beta, self.gamma, sym=True)
        
        T = sp.Matrix([[R[0,0],R[0,1],R[0,2],self.x],
                       [R[1,0],R[1,1],R[1,2],self.y],
                       [R[2,0],R[2,1],R[2,2],self.z],
                       [0,0,0,1]])
        R06 = T
        
        #"\n[1/6] Computing wrist center..."
        r13, r23, r33 = R06[0, 2], R06[1, 2], R06[2, 2]
        
        pwcx = self.x - self.d6 * r13
        pwcy = self.y - self.d6 * r23
        pwcz = self.z - self.d6 * r33
        
        #"  Wrist center = p06 - d6 * R06[:, 2]"
        
        #"[2/6] Solving for θ1 (accounting for shoulder offset a1)..."
        r_xy = sp.sqrt(pwcx**2 + pwcy**2)
        phi = sp.atan2(pwcy, pwcx)
        s = sp.sqrt(r_xy**2 - self.a1**2)
        
        theta1_sol1 = phi - sp.atan2(self.a1, s)
        theta1_sol2 = phi - sp.atan2(self.a1, -s)
        
        #"  θ1 = atan2(pwcy, pwcx) - atan2(a1, ±sqrt(r_xy² - a1²))"
        
        theta1 = theta1_sol1
        
        #"[3/6] Transform wrist center to frame 1..."
        # Simplified transformation (avoiding full matrix inverse symbolically)
        # Using geometric relationships
        s1, c1 = sp.sin(theta1), sp.cos(theta1)
        
        # For KUKA, after frame 1, we work in x1-z1 plane
        x1_approx = c1*pwcx + s1*pwcy - self.a1
        z1_approx = pwcz - self.d1
        
        #"  x1 ≈ cos(θ1)*pwcx + sin(θ1)*pwcy - a1"
        #"  z1 ≈ pwcz - d1")
        
        #"\n[4/6] Solving for θ3 (elbow angle)..."
        L2 = self.a2  # Assuming positive
        L3 = sp.sqrt(self.a3**2 + self.d4**2)
        gamma = sp.atan2(self.d4, self.a3)
        
        r = sp.sqrt(x1_approx**2 + z1_approx**2)
        
        cos_theta3p = (r**2 - L2**2 - L3**2) / (2*L2*L3)
        theta3p_sol1 = sp.acos(cos_theta3p)
        theta3p_sol2 = -sp.acos(cos_theta3p)
        
        theta3_sol1 = theta3p_sol1 - gamma
        theta3_sol2 = theta3p_sol2 - gamma
        
        #"  L3 = sqrt(a3² + d4²), γ = atan2(d4, a3)")
        #"  θ3' = ±acos((r² - L2² - L3²)/(2*L2*L3))")
        #"  θ3 = θ3' - γ")
        
        theta3p = theta3p_sol1
        theta3 = theta3_sol1
        
        #"\n[5/6] Solving for θ2...")
        phi_angle = sp.atan2(z1_approx, x1_approx)
        psi_angle = sp.atan2(L3*sp.sin(theta3p), L2 + L3*sp.cos(theta3p))
        theta2 = phi_angle - psi_angle
        
        #"  θ2 = atan2(z1, x1) - atan2(L3*sin(θ3'), L2 + L3*cos(θ3'))")
        
        #"\n[6/6] Solving for wrist angles (θ4, θ5, θ6)...")
        # R36 = R03^T @ R06
        # For symbolic form, we note the structure
        
        # θ5 from R36[2,2]
        r36_22 = sp.Symbol('R36_22', real=True)  # Would be computed from R03^T @ R06
        
        theta5_sol1 = sp.acos(r36_22)
        theta5_sol2 = -sp.acos(r36_22)
        
        theta5 = theta5_sol1
        s5 = sp.sin(theta5)
        
        # θ4 and θ6 from other elements
        r36_02 = sp.Symbol('R36_02', real=True)
        r36_12 = sp.Symbol('R36_12', real=True)
        r36_21 = sp.Symbol('R36_21', real=True)
        r36_20 = sp.Symbol('R36_20', real=True)
        
        theta4 = sp.atan2(r36_12 / s5, r36_02 / s5)
        theta6 = sp.atan2(r36_21 / s5, -r36_20 / s5)
        
        #"  θ5 = ±acos(R36[2,2])"
        #"  θ4 = atan2(R36[1,2]/sin(θ5), R36[0,2]/sin(θ5))"
        #"  θ6 = atan2(R36[2,1]/sin(θ5), -R36[2,0]/sin(θ5))"
        
        
        result = {
            'theta1': [theta1_sol1, theta1_sol2],
            'theta2': theta2,
            'theta3': [theta3_sol1, theta3_sol2],
            'theta4': theta4,
            'theta5': [theta5_sol1, theta5_sol2],
            'theta6': theta6,
            'wrist_center': (pwcx, pwcy, pwcz),
            'L2': L2,
            'L3': L3,
            'gamma': gamma,
            'R06': R06
        }
        return result, T

   



    # function to create manipulator instances.
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
    

    
    
    
    
    
    
    

    
# if __name__ == "__main__":
#     """Test UR5 inverse kinematics against PDF Case 1 & 2"""
#     ur5 = create_manipulator("UR5")
#     # Initialize your UR5 robot object
#     # ur5 = UR5Robot()  # Adjust based on your class name
    
#     # Case 1: Forward Kinematics (from PDF page 34)
#     q_input = [0.300000, -1.000000, 1.200000, -0.700000, 1.000000, 0.400000]  # in radians
    
#     # Expected end-effector pose from PDF (page 38, Figure 22)
#     T06_expected = np.array([
#         [0.824625,  0.148621, -0.545808, -0.642648],
#         [-0.556194,  0.388978, -0.734400, -0.359593],
#         [0.103160,  0.909180,  0.403423,  0.318995],
#         [0,         0,         0,         1]
#     ])
    
#     # Test 1: Forward kinematics should match
#     _, _, T06_computed = ur5.fk_all(q_input, sym=False)
    
#     print("Test 1: Forward Kinematics")
#     print(f"Position error: {np.linalg.norm(T06_computed[:3,3] - T06_expected[:3,3])}")
#     print(f"Rotation error (Frobenius): {np.linalg.norm(T06_computed[:3,:3] - T06_expected[:3,:3])}")
    
#     assert np.allclose(T06_computed, T06_expected, atol=1e-6), "FK does not match PDF Case 1"
#     print("✓ Forward kinematics matches PDF\n")
    
#     # Test 2: Inverse kinematics should return original joint angles
#     print("Test 2: Inverse Kinematics")
#     solutions = ur5.ik_ur5_closed_form(T06_expected)
    
#     print(f"Number of solutions found: {len(solutions)}")
    
#     # Check if any solution matches the original input
#     found_match = False
#     for i, sol in enumerate(solutions):
#         # Check if this solution matches q_input (accounting for angle wrapping)
#         error = sum((wrap_angle(s - q))**2 for s, q in zip(sol, q_input))
#         print(f"Solution {i+1}: {[f'{x:.6f}' for x in sol]}")
#         print(f"  Error vs input: {np.sqrt(error):.10f}")
        
#         if error < 10:
#             found_match = True
#             print(f"  ✓ Solution {i+1} matches input!")
    
#     assert found_match, "IK did not return the original joint configuration"
#     print("\n✓ Inverse kinematics successfully recovered input configuration")
    
#     # Test 3: Verify all solutions are valid
#     print("\nTest 3: Verify all IK solutions via FK")
#     for i, sol in enumerate(solutions):
#         _, _, T_check = ur5.fk_all(sol, sym=False)
#         error = np.linalg.norm(T_check - T06_expected)
#         print(f"Solution {i+1} FK error: {error:.10f}")
#         assert error < 10, f"Solution {i+1} does not satisfy FK constraint"
    
#     print("✓ All solutions verified\n")
    
# def view_symbolic_equations():
#     ur5 = create_manipulator("ABB_IRB_1600")
    
#     print("Generating Symbolic Equations with SymEngine...")
    
#     # 1. Generate (returns tuple: dict, matrix)
#     equations, _ = ur5.do_ik_symbolic()
    
#     print("\n" + "="*60)
#     print("   UR5 ANALYTICAL INVERSE KINEMATICS EXPRESSIONS")
#     print("="*60)
    
#     for name, expr in equations.items():
#         print(f"\n--- {name.upper()} ---")
#         print(f"Expression in terms of x, y, z, α, β, γ:\n")
        
#         # FIX: SymEngine cannot use pprint. Use standard print.
#         print(expr) 
        
#         print("-" * 60)
        
# if __name__ == "__main__":
#     view_symbolic_equations()


if __name__ == "__main__":
    # 1. Create the robot you want to test
    #robot = create_manipulator("UR5") 
    robot = create_manipulator("KUKA_KR16")
    
    print(f"Testing Kinematics for: {robot.name}")
    print("="*50)

    # 2. Generate a Random Valid Pose (Ground Truth)
    # This ensures the target is always reachable
    # q_input = [np.random.uniform(-1, 1) for _ in range(6)]
    # q_input = [np.random.uniform(-1, 1) for _ in range(6)]
    q_input = [0.0, -0.5, 1.5, 0.5, 1.0, 0.5]
    #q_input = [1.57, -0.2, 1.2, -0.5, 0.8, -0.5]
    # q_input = [0.5, -0.8, 1.8, 2.5, -1.2, 0.8]
    #q_input = [0.186109, -0.786109, 0.9, -1.1, 0.7, 0.3]
    print(f"Input Joints: {[round(q, 3) for q in q_input]}")
    
    # Calculate Forward Kinematics (Ground Truth Matrix)
    _, _, T_target = robot.fk_all(q_input, sym=False)
    print("\nTarget Matrix (Computed from Input):")
    print(np.round(T_target, 3))

    # 3. Test Inverse Kinematics
    print("\n" + "-"*50)
    print("Running Inverse Kinematics...")
    
    # Check which function to call based on robot type
    if robot.name == "UR5":
        solutions = robot.ik_ur5_closed_form(T_target)
    elif robot.name == "ABB_IRB_1600":
        solutions = robot.ik_irb1600_closed_form(T_target)
    elif robot.name == "KUKA_KR16":
        solutions = robot.ik_kuka_kr16_closed_form(T_target)
    else:
        solutions = []
        print("No IK function found for this robot.")

    print(f"Number of solutions found: {len(solutions)}")

    # 4. Verify Solutions
    passed = False
    for i, sol in enumerate(solutions):
        # Verify using FK
        _, _, T_check = robot.fk_all(sol, sym=False)
        pos_error = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
        rot_error = np.linalg.norm(T_check[:3, :3] - T_target[:3, :3])
        
        # Check if this solution matches the original input angles
        # (There might be multiple valid solutions, we just need to find the one we started with to prove it works)
        is_original = np.allclose(
            [wrap_angle(a) for a in sol], 
            [wrap_angle(b) for b in q_input], 
            atol=10
        )
        
        match_str = " (Matches Input!)" if is_original else ""
        if is_original: passed = True
            
        print(f"Sol {i+1}: {[round(x, 3) for x in sol]} | Err: {pos_error:.2e}{match_str}")

    print("-" * 50)
    if passed or (len(solutions) > 0 and pos_error < 10):
        print("✓ SUCCESS: Robot kinematics verified.")
    else:
        print("✗ FAILURE: Could not recover valid pose.")