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
    

        # Compute forward kinematics numerically       
        # Args:
        #     joint_values: List of joint values (in radians for revolute, meters for prismatic)
        #     frame_range: Tuple (start, end) or None for all frames   
        # Returns:
        #     (transforms, final_transform) where transforms is list of 4x4 transformation matrices

    # def compute_fk_numeric(self, joint_values, frame_range=None):
    #     if len(joint_values) != self.num_joints:
    #         raise ValueError(f"Expected {self.num_joints} joint values, got {len(joint_values)}")
        
    #     transforms = []
    #     T = np.eye(4)
        
    #     for i, (params, q_val) in enumerate(zip(self._dh_params, joint_values)):
    #         # Get DH parameters
    #         if params['variable'] == 'theta':
    #             theta = q_val
    #             d = params['d']
    #         else:  # prismatic joint
    #             theta = params['theta']
    #             d = q_val
            
    #         a = params['a']
    #         alpha = params['alpha']
            
    #         # Compute transformation matrix using DH convention
    #         A = self._T_matrix(theta, d, a, alpha)
    #         T = T @ A
    #         transforms.append(T.copy())
        
    #     # Apply frame range filter if specified
    #     if frame_range is not None:
    #         start, end = frame_range
    #         transforms = transforms[start:end+1]
        
    #     return transforms, T
    
    #     # Compute forward kinematics symbolically
    #     # Args:
    #     #     frame_range: Tuple (start, end) or None for all frames  
    #     # Returns:
    #     #     (transforms, final_transform, joint_symbols) as symbolic matrices
    # def compute_fk_symbolic(self, frame_range=None):
    #     # Create symbolic joint variables
    #     q_symbols = []
    #     for i, params in enumerate(self._dh_params):
    #         if params['variable'] == 'theta':
    #             q_symbols.append(sp.Symbol(f'θ{i+1}', real=True))
    #         else:
    #             q_symbols.append(sp.Symbol(f'd{i+1}', real=True))
        
    #     transforms = []
    #     T = sp.eye(4)
        
    #     for i, params in enumerate(self._dh_params):
    #         # Get DH parameters
    #         if params['variable'] == 'theta':
    #             theta = q_symbols[i]
    #             d = params['d']
    #         else:
    #             theta = params['theta']
    #             d = q_symbols[i]
            
    #         a = params['a']
    #         alpha = params['alpha']
            
    #         # Compute symbolic transformation matrix
    #         A = self._T_matrix_symbolic(theta, d, a, alpha)
    #         T = T * A
    #         T = sp.simplify(T)
    #         transforms.append(T.copy())
        
    #     # Apply frame range filter if specified
    #     if frame_range is not None:
    #         start, end = frame_range
    #         transforms = transforms[start:end+1]
        
    #     return transforms, T, q_symbols

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
        
    # def compute_ik_numeric(self, target_matrix):
    #     """
    #     Compute inverse kinematics numerically
    #     Override this in child classes with specific IK solutions
        
    #     Args:
    #         target_matrix: 4x4 target transformation matrix
            
    #     Returns:
    #         List of solution arrays, each containing joint values
    #     """
    #     raise NotImplementedError(f"IK not yet implemented for {self.name}")
    
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
    def do_ik_symbolic(self):
   
        #x,y,z,alpha,beta,gamma = sp.symbols(" ".join(names), real=True)
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
 
    


    # def do_ik(self):
    #     comp_mode = self.sym_num_widget.currentRow()
    #     sym = (comp_mode == 0)
        
    #     if sym:
    #         do_ik_symbolic(self)
    #         return

    #     print("\nInput: 1) 4×4 matrix  2) x,y,z,α,β,γ")
    #     ch = ask_choice("Choose: ", ["1","2"], "1")
    #     T = read_T_matrix() if ch == "1" else read_pose()

    #     print("\nTarget 0T6 =")
    #     pprint("0T6", T, sym=False)

    #     print("\nSolving...")

    #     sols = ik_ur5_closed_form(robot, T)

    #     print("\n======================================================================")
    #     print("RESULTS")
    #     print("======================================================================")
    #     if not sols:
    #         print("No solution found.")
    #         print("\n======================================================================")
    #         print("Done!")
    #         print("======================================================================\n")
    #         return

    #     print(f"Raw: {len(sols)} | Valid: {len(sols)}")
    #     print("\n--- Solutions (rad) ---")
    #     for i,s in enumerate(sols, start=1):
    #         print(f"{i:02d}: [{', '.join(f'{v:+.6f}' for v in s)}]")

    #     print("\n--- Solutions (deg) ---")
    #     for i,s in enumerate(sols, start=1):
    #         sd = [r2d(v) for v in s]
    #         print(f"{i:02d}: [{', '.join(f'{v:+.3f}' for v in sd)}]")

    #     print("\n======================================================================")
    #     print("Done!")
    #     print("======================================================================\n")
    





















        
#     def compute_ik_numeric(self, T06):
#         """
#         Closed-form IK solution for UR5 (6-DOF manipulator)
#         Based on geometric approach - consistent with FK
        
#         Args:
#             T06: 4x4 target transformation matrix
            
#         Returns:
#             List of valid joint configurations (each is a list of 6 joint angles in radians)
#         """
#         dh = self._dh_params
        
#         # Extract UR5 DH parameters
#         d1 = dh[0]['d']
#         a2 = dh[1]['a']
#         a3 = dh[2]['a']
#         d4 = dh[3]['d']
#         d5 = dh[4]['d']
#         d6 = dh[5]['d']
        
#         R06 = T06[:3, :3]
#         p06 = T06[:3, 3]
#         px, py, pz = float(p06[0]), float(p06[1]), float(p06[2])
        
#         solutions = []
        
#         # Calculate wrist center (p05)
#         p05 = p06 - d6 * R06[:, 2]
#         rxy = np.hypot(p05[0], p05[1])
        
#         if rxy < 1e-12:
#             return []
        
#         # θ1 (two solutions)
#         c = d4 / rxy
#         if abs(c) > 1.0 + 1e-9:
#             return []
#         c = max(-1.0, min(1.0, c))
        
#         phi = np.arccos(c)
#         psi = np.arctan2(p05[1], p05[0])
#         q1_candidates = [
#             self._wrap_angle(psi + phi + np.pi/2),
#             self._wrap_angle(psi - phi + np.pi/2)
#         ]
        
#         for q1 in q1_candidates:
#             s1, c1 = np.sin(q1), np.cos(q1)
            
#             # θ5 (two solutions)
#             c5 = (px*s1 - py*c1 - d4) / d6
#             if abs(c5) > 1.0 + 1e-9:
#                 continue
#             c5 = max(-1.0, min(1.0, c5))
            
#             q5a = np.arccos(c5)
#             q5_candidates = [self._wrap_angle(q5a), self._wrap_angle(-q5a)]
            
#             for q5 in q5_candidates:
#                 s5 = np.sin(q5)
#                 if abs(s5) < 1e-10:
#                     continue
                
#                 # θ6
#                 q6 = np.arctan2(
#                     (-R06[0,1]*s1 + R06[1,1]*c1) / s5,
#                     (R06[0,0]*s1 - R06[1,0]*c1) / s5
#                 )
#                 q6 = self._wrap_angle(q6)
                
#                 # Compute T14 = inv(T01)*T06*inv(T45*T56)
#                 T01 = self._T_matrix(q1, d1, 0.0, np.pi/2)
#                 T45 = self._T_matrix(q5, d5, 0.0, -np.pi/2)
#                 T56 = self._T_matrix(q6, d6, 0.0, 0.0)
                
#                 T14 = np.linalg.inv(T01) @ T06 @ np.linalg.inv(T45 @ T56)
                
#                 # θ2, θ3 from planar geometry in frame 1
#                 p14 = T14[:3, 3]
#                 x, y = float(p14[0]), float(p14[1])
                
#                 D = (x*x + y*y - a2*a2 - a3*a3) / (2*a2*a3)
#                 if abs(D) > 1.0 + 1e-9:
#                     continue
#                 D = max(-1.0, min(1.0, D))
                
#                 for q3 in [np.arccos(D), -np.arccos(D)]:
#                     q2 = np.arctan2(y, x) - np.arctan2(a3*np.sin(q3), a2 + a3*np.cos(q3))
#                     q2, q3 = self._wrap_angle(q2), self._wrap_angle(q3)
                    
#                     # θ4 from rotation
#                     qtemp = [q1, q2, q3, 0.0, 0.0, 0.0]
#                     Ts, _ = self.compute_fk_numeric(qtemp)
#                     R03 = Ts[2][:3, :3]
#                     R04 = (T01 @ T14)[:3, :3]
#                     R34 = R03.T @ R04
#                     q4 = self._wrap_angle(np.arctan2(R34[1,0], R34[0,0]))
                    
#                     candidate = [
#                         self._wrap_angle(q1),
#                         self._wrap_angle(q2),
#                         self._wrap_angle(q3),
#                         self._wrap_angle(q4),
#                         self._wrap_angle(q5),
#                         self._wrap_angle(q6)
#                     ]
                    
#                     # Validate by FK to guarantee consistency
#                     _, Tchk = self.compute_fk_numeric(candidate)
#                     if np.allclose(Tchk, T06, atol=1e-6, rtol=0):
#                         solutions.append(candidate)
        
#         # Remove duplicates (wrap-aware)
#         unique_solutions = []
#         for s in solutions:
#             is_duplicate = False
#             for u in unique_solutions:
#                 if sum((self._wrap_angle(si-ui))**2 for si, ui in zip(s, u)) < 1e-10:
#                     is_duplicate = True
#                     break
#             if not is_duplicate:
#                 unique_solutions.append(s)
        
#         return unique_solutions


















#ABB IRB1600 industrial manipulator
class ABB_IRB_1600(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "ABB_IRB_1600"
    
        # ABB IRB1600 DH parameters
    def _initialize_dh_parameters(self):
        a1 = 0.150
        d1 = 0.4865
        a2 = 0.700
        a3 = 0.115
        d4 = 0.7200
        d6 = 0.0850
        return [
            {"a": a1, "alpha": pi / 2,  "d": d1, "theta": 0.0, "variable": "theta"},
            {"a": a2, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a3, "alpha": pi / 2 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2, "d": d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": 0, "d": d6, "theta": 0.0, "variable": "theta"},
        ]




class KUKA_KR16(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "KUKA_KR16"
    
        # KUKA KR16 DH parameters
    def _initialize_dh_parameters(self):
        a2 = 0.675
        a3 = 0.680
        a4 = 0.670
        a6 = 0.115
        d2 = 0.260
        d4 = 0.035
        return [
            {"a": 0, "alpha": 0 ,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a2, "alpha":  pi / 2 , "d": d2, "theta": 0.0, "variable": "theta"},
            {"a": a3, "alpha": 0 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a4, "alpha":  pi / 2, "d": d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a6, "alpha":  pi / 2, "d": 0, "theta": 0.0, "variable": "theta"},
        ]




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