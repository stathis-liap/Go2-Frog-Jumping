import numpy as np

def get_rotation_matrix(roll, pitch):
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    return Ry @ Rx

def calculate_vmc_forces(R, P_base):
    k_att = 200.0  
    P = R @ P_base
    F_VMC_world = np.zeros((3, 4))
    
    F_VMC_world[2, :] = k_att * P[2, :]
    
    F_VMC_body = R.T @ F_VMC_world
    return F_VMC_body

class LegController:
    def __init__(self):
        self.Kp_cartesian = 400.0 * np.eye(3) 
        self.Kd_cartesian = 8.0 * np.eye(3)
        self.Kd_joint = 0.8 * np.eye(3)
        self.v_desired = np.array([0.0, 0.0, 0.0])

    def compute_total_torque(self, J, F_jump, p_current, p_desired, v_current, dq_current, F_vmc_leg_body, R):
        tau_f = J.T @ F_jump 
        
        p_error = p_desired - p_current
        v_error = self.v_desired - v_current
        
        # F_gravity_world = np.array([0.0, 0.0, -36.75])
        # F_gravity_body = R.T @ F_gravity_world
        
        F_imp = self.Kp_cartesian @ p_error + self.Kd_cartesian @ v_error #+ F_gravity_body
        tau_imp = J.T @ F_imp - (self.Kd_joint @ dq_current)
        
        tau_vmc = J.T @ F_vmc_leg_body 
        
        return tau_f + tau_imp + tau_vmc