import numpy as np

class Go2Kinematics:
    def __init__(self):
        self.l1 = 0.0955  # Hip link
        self.l2 = 0.213   # Thigh link
        self.l3 = 0.213   # Calf link

    def compute_leg_kinematics(self, q, leg_id):
        """
        Calculates the Foot Position (p) and Jacobian Matrix (J) for a given leg.
        q: array of 3 joint angles [hip_roll, hip_pitch, knee_pitch]
        leg_id: 0: FR, 1: FL, 2: RR, 3: RL
        """
        q1, q2, q3 = q[0], q[1], q[2]
        
        # Left legs (FL, RL) have their hips mirrored compared to Right legs
        side_sign = 1 if leg_id in [1, 3] else -1
        l1 = self.l1 * side_sign

        # Pre-compute sines and cosines
        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

        # Forward Kinematics
        px = -self.l3 * s23 - self.l2 * s2
        py = self.l3 * c23 * s1 + self.l2 * c2 * s1 + l1 * c1
        pz = -self.l3 * c23 * c1 - self.l2 * c2 * c1 + l1 * s1
        p = np.array([px, py, pz])

        # Analytical Jacobian Matrix
        J = np.zeros((3, 3))
        
        # d(px) / d(q)
        J[0, 0] = 0.0
        J[0, 1] = -self.l3 * c23 - self.l2 * c2
        J[0, 2] = -self.l3 * c23
        
        # d(py) / d(q)
        J[1, 0] = self.l3 * c23 * c1 + self.l2 * c2 * c1 - l1 * s1
        J[1, 1] = -self.l3 * s23 * s1 - self.l2 * s2 * s1
        J[1, 2] = -self.l3 * s23 * s1
        
        # d(pz) / d(q)
        J[2, 0] = self.l3 * c23 * s1 + self.l2 * c2 * s1 + l1 * c1
        J[2, 1] = self.l3 * s23 * c1 + self.l2 * s2 * c1
        J[2, 2] = self.l3 * s23 * c1

        return p, J