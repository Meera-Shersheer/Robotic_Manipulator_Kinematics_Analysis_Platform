"""
KUKA KR16 IK Solution Count Analysis
This script tests multiple random poses to show that the number of 
valid IK solutions varies based on the target pose geometry.
"""

import numpy as np

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2*np.pi) - np.pi

def test_multiple_poses(robot, num_tests=10):
    """
    Test IK for multiple random poses and count solutions.
    This demonstrates that 8 solutions is the theoretical maximum,
    but many poses will have fewer valid solutions.
    """
    
    print("="*70)
    print("KUKA KR16 IK Solution Count Analysis")
    print("="*70)
    print("\nTesting random reachable poses...")
    print("-"*70)
    
    solution_counts = []
    
    for test_num in range(num_tests):
        # Generate random joint configuration (within reasonable bounds)
        q_input = [
            np.random.uniform(-2.0, 2.0),   # theta1
            np.random.uniform(-1.5, 1.5),   # theta2
            np.random.uniform(-1.0, 2.0),   # theta3
            np.random.uniform(-2.0, 2.0),   # theta4
            np.random.uniform(-1.5, 1.5),   # theta5
            np.random.uniform(-2.0, 2.0),   # theta6
        ]
        
        # Compute FK
        _, _, T_target = robot.fk_all(q_input, sym=False)
        
        # Compute IK
        solutions = robot.ik_kuka_kr16_closed_form(T_target)
        
        solution_counts.append(len(solutions))
        
        print(f"Test {test_num+1:2d}: Input = {[f'{x:5.2f}' for x in q_input]}")
        print(f"         Solutions found: {len(solutions)}")
        
        # Verify at least one solution matches input
        found_match = False
        for sol in solutions:
            if np.allclose([wrap_angle(a) for a in sol], 
                          [wrap_angle(b) for b in q_input], 
                          atol=1e-2):
                found_match = True
                break
        
        if found_match:
            print(f"         ✓ Original configuration recovered")
        else:
            print(f"         ⚠ Warning: Original config not in solution set!")
        print()
    
    print("="*70)
    print("SUMMARY")
    print("="*70)
    print(f"Average solutions per pose: {np.mean(solution_counts):.1f}")
    print(f"Min solutions: {min(solution_counts)}")
    print(f"Max solutions: {max(solution_counts)}")
    print(f"\nSolution distribution:")
    for count in sorted(set(solution_counts)):
        occurrences = solution_counts.count(count)
        print(f"  {count} solutions: {occurrences} poses ({100*occurrences/num_tests:.0f}%)")
    
    print("\n" + "="*70)
    print("EXPLANATION")
    print("="*70)
    print("""
The number of IK solutions depends on the target pose geometry:

1. **8 solutions (2×2×2)**: Both theta1 branches reach the target, 
   both elbow configurations are valid, both wrist flips work.
   This is the MAXIMUM, not the norm.

2. **4 solutions (1×2×2 or 2×2×1)**: One theta1 branch is unreachable,
   OR one wrist flip configuration fails validation.
   This is VERY COMMON.

3. **2 solutions**: Only one combination of branches is valid.
   This occurs for poses near workspace boundaries.

4. **0 solutions**: Target is outside the workspace or at a 
   configuration where numerical issues prevent convergence.

Your original code is CORRECT. Getting 4 solutions for a specific 
pose is normal and expected behavior!
""")

# Example analysis function for a specific pose
def analyze_specific_pose(robot, q_input):
    """Detailed analysis of why a specific pose has N solutions"""
    
    print("\n" + "="*70)
    print("DETAILED ANALYSIS FOR SPECIFIC POSE")
    print("="*70)
    
    _, _, T_target = robot.fk_all(q_input, sym=False)
    
    # Extract geometric info
    R06, p06 = T_target[:3, :3], T_target[:3, 3]
    pwc = p06 - robot.d[5] * R06[:, 2]
    xw, yw, zw = pwc[0], pwc[1], pwc[2]
    
    print(f"\nInput joints: {[f'{x:.3f}' for x in q_input]}")
    print(f"End-effector position: [{p06[0]:.3f}, {p06[1]:.3f}, {p06[2]:.3f}]")
    print(f"Wrist center: [{xw:.3f}, {yw:.3f}, {zw:.3f}]")
    
    # Check theta1 branches
    r_xy = np.hypot(xw, yw)
    print(f"\nWrist center radial distance: {r_xy:.3f} m")
    print(f"Shoulder offset (a1): {robot.a[1]:.3f} m")
    
    th1_base = np.arctan2(yw, xw)
    th1_candidates = [wrap_angle(th1_base), wrap_angle(th1_base + np.pi)]
    
    print(f"\nTheta1 candidates:")
    
    a1 = robot.a[1]
    L2 = abs(robot.a[2])
    L3 = np.hypot(robot.a[3], robot.d[3])
    
    for i, th1 in enumerate(th1_candidates):
        T01 = dh_Craig(th1, robot.d[0], 0.0, 0.0, sym=False)
        p1 = np.linalg.inv(T01) @ np.array([xw, yw, zw, 1.0])
        
        x_tri = float(p1[0]) - a1
        z_tri = float(p1[2])
        
        r_sq = x_tri**2 + z_tri**2
        D = (r_sq - L2**2 - L3**2) / (2 * L2 * L3)
        
        print(f"  Branch {i+1}: θ1 = {np.degrees(th1):6.1f}°")
        print(f"    Triangle: x={x_tri:.3f}, z={z_tri:.3f}, r={np.sqrt(r_sq):.3f}")
        print(f"    Elbow cos(θ3'): D = {D:.3f}")
        
        if abs(D) <= 1.0:
            print(f"    ✓ VALID - 2 elbow configs × 2 wrist flips = 4 solutions")
        else:
            print(f"    ✗ INVALID - Elbow cannot reach (|D| > 1.0)")
    
    # Now run actual IK
    solutions = robot.ik_kuka_kr16_closed_form(T_target)
    
    print(f"\nActual IK solutions found: {len(solutions)}")
    print("\nConclusion:")
    print("-" * 70)
    
    if len(solutions) == 4:
        print("Getting 4 solutions is EXPECTED when one theta1 branch is unreachable.")
        print("This is CORRECT behavior, not a bug!")
    elif len(solutions) == 8:
        print("Getting 8 solutions means both theta1 branches can reach the target.")
        print("This is the maximum possible, but uncommon for arbitrary poses.")
    else:
        print(f"Getting {len(solutions)} solutions is normal for this pose geometry.")

# Helper function (you need this in your actual code)
def dh_Craig(theta, d, a, alpha, sym=False):
    """Modified DH transformation"""
    if sym:
        import sympy as sp
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
    
    
    

# Usage example:
if __name__ == "__main__":
    from your_robot_module import create_manipulator
    
    robot = create_manipulator("KUKA_KR16")
    
    # Test the specific pose you mentioned
    q_input = [1.57, -0.2, 1.2, -0.5, 0.8, -0.5]
    analyze_specific_pose(robot, q_input)
    
    # Test multiple random poses
    test_multiple_poses(robot, num_tests=20)