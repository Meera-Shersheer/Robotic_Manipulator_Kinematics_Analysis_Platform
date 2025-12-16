from imports import *
 
#ABB IRB2600 industrial manipulator
class ABB_IRB_2600(RoboticManipulator):
    
    def __init__(self):
        super().__init__()
        self.name = "ABB IRB_2600"
    
        # ABB IRB2600 DH parameters
    def _initialize_dh_parameters(self):
        a1 = 0.150
        d1 = 0.445
        a2 = 0.700
        a3 = 0.115
        d4 = 0.7950
        d6 = 0.0850
        return [
            {"a": a1, "alpha": pi / 2,  "d": d1, "theta": 0.0, "variable": "theta"},
            {"a": a2, "alpha": 0, "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": a3, "alpha": pi / 2 , "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": -pi / 2, "d": d4, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": pi / 2,  "d": 0, "theta": 0.0, "variable": "theta"},
            {"a": 0, "alpha": 0, "d": d6, "theta": 0.0, "variable": "theta"},
        ]

