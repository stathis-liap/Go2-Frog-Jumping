import numpy as np

# --- Cartesian PD Impedance Gains ---
KP_CARTESIAN = 400.0  
KD_CARTESIAN = 10.0        
KD_JOINT = 0.8            

# --- Virtual Model Control Gains ---
K_ATTITUDE = 550.0  

# --- Optimization Boundaries ---
BOUNDS_F0 = (1.0, 4.75)   # Hz
BOUNDS_FX = (0.0, 250.0)  # N
BOUNDS_FY = (-250.0, 250.0)  # N
BOUNDS_FZ = (250.0, 450.0)   # N