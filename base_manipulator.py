from gui.imports import *



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
    
    # Reset all joint values to zero (home position)
    # def reset_joint_values(self):
    #     for param in self._dh_params:
    #         if param['variable'] == 'theta':
    #             param['theta'] = 0.0
    #         elif param['variable'] == 'd':
    #             param['d'] = 0.0
    
    # def get_dh_table_for_display(self, angle_in_degrees=False):
    #     """
    #     Get DH parameters formatted for display in the table.
        
    #     Args:
    #         angle_in_degrees: If True, return theta/alpha in degrees
            
    #     Returns:
    #         List of dicts with formatted values
    #     """
    #     display_params = []
        
    #     for i, param in enumerate(self._dh_params):
    #         display_param = {
    #             'joint': i + 1,
    #             'theta': param['theta'],
    #             'd': param['d'],
    #             'a': param['a'],
    #             'alpha': param['alpha'],
    #             'variable': self.get_joint_variable_name(i),
    #             'value': self.get_joint_value(i, angle_in_degrees)
    #         }
            
    #         # Convert theta to degrees if requested
    #         if angle_in_degrees and param['variable'] == 'theta':
    #             display_param['theta'] = np.rad2deg(param['theta'])
            
    #         display_params.append(display_param)
        
    #     return display_params



class UR5(RoboticManipulator):
    """Universal Robots UR5 manipulator"""
    
    def __init__(self):
        super().__init__()
        self.name = "UR5"
    
    #   UR5 DH parameters (Modified DH convention)
    def _initialize_dh_parameters(self) :
        d1 = 0.089159
        a2= -0.425
        a3 = -0.39225
        d4 = 0.10915
        d5 = 0.09465
        d6 = 0.0823
        return [
            {"a": 0, "alpha": pi / 2,  "d": d1, "theta": 0.0, "variable": "theta"},
            {"a": a2, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a3, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2, "d": d5, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": 0,   "d": d6, "theta": 0.0, "variable": "theta"},
        ]



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