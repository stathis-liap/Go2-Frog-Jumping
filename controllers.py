import numpy as np
import config

def get_rotation_matrix(roll, pitch):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll),  np.cos(roll)]])
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [0,              1, 0             ],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    return Ry @ Rx


def calculate_vmc_forces(R, P_base):
    k_att = config.K_ATTITUDE
    
    # [OPTIONAL UPGRADE]: True metric dimensions for the Unitree Go2 hips
    Lx = 1.0  # meters from center to front/back hips
    Ly = 1.0  # meters from center to left/right hips
    
    P_base_true = np.array([
        [ Lx,  Lx, -Lx, -Lx],
        [-Ly,  Ly, -Ly,  Ly],
        [  0,   0,   0,   0]
    ])

    P = R @ P_base_true  

    F_VMC_world = np.zeros((3, 4))
    
    # Negative sign ensures it pushes against the floor
    F_VMC_world[2, :] = k_att * (np.array([0, 0, 1]) @ P)  

    return R.T @ F_VMC_world


class LegController:
    def __init__(self):
        # Assumes config variables are scalar floats, e.g., 400.0
        self.Kp       = config.KP_CARTESIAN * np.eye(3)
        self.Kd       = config.KD_CARTESIAN * np.eye(3)
        self.Kd_joint = config.KD_JOINT     * np.eye(3)

    def compute_total_torque(self, J, F_jump, p_current, p_desired,
                             v_current, dq_current, F_vmc_leg_body, R):
        
        # Eq (3): feed-forward jump thrust
        tau_f = J.T @ F_jump

        # Eq (4): Cartesian PD impedance — desired foot velocity is zero
        tau_imp = J.T @ (self.Kp @ (p_desired - p_current)
                         - self.Kd @ v_current) \
                  - self.Kd_joint @ dq_current

        # Eq (7): VMC attitude correction
        tau_vmc = J.T @ F_vmc_leg_body

        # Eq (8)
        return tau_f + tau_imp + tau_vmc