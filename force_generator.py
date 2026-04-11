import numpy as np

class JumpForceGenerator:
    def __init__(self, dt=0.001):
        self.dt = dt
        self.theta = 0.0

    def reset(self):
        self.theta = np.pi + 0.001 

    def step(self, f0, f1, Fx, Fy, Fz):
        f = f1 if self.theta < np.pi else f0
        self.theta += 2.0 * np.pi * f * self.dt
        
        done = False
        if self.theta >= 2.0 * np.pi:
            self.theta = 0.0
            done = True 

        sin_theta = np.sin(self.theta)
        if sin_theta < 0 and not done:
            F_foot = np.array([-Fx, Fy, -Fz]) * (-sin_theta)  
        else:
            F_foot = np.zeros(3)

        return F_foot, self.theta, done