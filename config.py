import numpy as np

# --- Cartesian PD Impedance Gains ---
KP_CARTESIAN = np.diag([400.0, 400.0, 400.0])  
KD_CARTESIAN = np.diag([8.0, 8.0, 8.0])        
KD_JOINT = np.diag([0.6, 0.6, 0.6])            

# --- Virtual Model Control Gains ---
K_ATTITUDE = 150.0  # k_att = 200

# --- Optimization Boundaries ---
BOUNDS_F0 = (1.75, 4.75)  # Hz
BOUNDS_FX = (0.0, 150.0)  # N
BOUNDS_FY = (0.0, 150.0)  # N
BOUNDS_FZ = (150.0, 350.0) # N
