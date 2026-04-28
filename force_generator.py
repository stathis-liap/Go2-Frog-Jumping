import numpy as np

class JumpForceGenerator:
    def __init__(self, dt=0.001):
        self.dt = dt
        # [UPGRADE]: The CPG now tracks 4 independent clocks, one for each leg!
        self.theta = np.zeros(4) 

    def reset(self):
        self.theta = np.full(4, np.pi + 0.001)

    def step(self, f0_arr, f1, Fx_arr, Fy_arr, Fz_arr):
        # f0_arr is now an array of 4 different frequencies
        f = np.where(self.theta < np.pi, f1, f0_arr)
        
        # Advance each leg's clock at its own unique speed
        self.theta += 2.0 * np.pi * f * self.dt

        # The overall jump is only "done" when the slowest leg finishes pushing
        done = np.all(self.theta >= 2.0 * np.pi)
        if done:
            self.theta = np.zeros(4)

        sin_theta = np.sin(self.theta)

        # Leg order: 0=FR, 1=FL, 2=RR, 3=RL
        side_signs = np.array([1.0, -1.0, 1.0, -1.0])

        forces = np.zeros((3, 4))
        
        for leg in range(4):
            # Only apply force if THIS specific leg is still in its push phase
            if sin_theta[leg] < 0 and self.theta[leg] < 2.0 * np.pi:
                forces[:, leg] = np.array([
                    Fx_arr[leg],
                    Fy_arr[leg] * side_signs[leg], 
                    Fz_arr[leg]
                ]) * (sin_theta[leg])

        return forces, self.theta, done