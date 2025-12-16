from imports import *

class UR5(RoboticManipulator):
    """Universal Robots UR5 manipulator"""
    
    def __init__(self):
        super().__init__()
        self.name = "UR5"
    
    #   UR5 DH parameters (Modified DH convention)
    def _initialize_dh_parameters(self) :
        d1 = 0.089159
        a2= 0.425
        a3 = 0.392
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

