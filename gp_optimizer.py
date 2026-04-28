from bayes_opt import BayesianOptimization
from bayes_opt.acquisition import UpperConfidenceBound
import config

class JumpOptimizer:
    def __init__(self, jump_type='forward', random_state=42):
        self.jump_type = jump_type
        self.pbounds = {}

        # Only inject the EXACT forces needed for the specific jump type
        if jump_type == 'forward':
            self.pbounds.update({
                'f0_front': config.BOUNDS_F0, 'f0_back': config.BOUNDS_F0,
                'Fx_front': config.BOUNDS_FX, 'Fz_front': config.BOUNDS_FZ,
                'Fx_back': config.BOUNDS_FX,  'Fz_back': config.BOUNDS_FZ
            })
        elif jump_type == 'lateral':
            self.pbounds.update({
                'f0_left': config.BOUNDS_F0,  'f0_right': config.BOUNDS_F0,
                'Fy_left': config.BOUNDS_FY,  'Fz_left': config.BOUNDS_FZ,
                'Fy_right': config.BOUNDS_FY, 'Fz_right': config.BOUNDS_FZ
            })
        elif jump_type == 'twist':
            self.pbounds.update({
                'f0_front': config.BOUNDS_F0, 'f0_back': config.BOUNDS_F0,
                'Fy_front': config.BOUNDS_FY, 'Fz_front': config.BOUNDS_FZ,
                'Fy_back': config.BOUNDS_FY,  'Fz_back': config.BOUNDS_FZ
            })

        self.acq = UpperConfidenceBound(kappa=1.5)
        self.optimizer = BayesianOptimization(
            f=None, 
            pbounds=self.pbounds,
            acquisition_function=self.acq,
            verbose=0, 
            random_state=random_state
        )
        
        self.iteration = 0
        
        print("\n" + "="*85)
        if self.jump_type == 'forward':
            print(f"{'Iter':>4} | {'Dist (m)':>8} | {'f0_Fr':>5} | {'f0_Bk':>5} | {'Fx_Fr':>6} | {'Fz_Fr':>6} | {'Fx_Bk':>6} | {'Fz_Bk':>6}")
        elif self.jump_type == 'twist':
            print(f"{'Iter':>4} | {'Yaw(rad)':>8} | {'f0_Fr':>5} | {'f0_Bk':>5} | {'Fy_Fr':>6} | {'Fz_Fr':>6} | {'Fy_Bk':>6} | {'Fz_Bk':>6}")
        elif self.jump_type == 'lateral':
            print(f"{'Iter':>4} | {'Dist (m)':>8} | {'f0_L':>5} | {'f0_R':>5} | {'Fy_L':>6} | {'Fz_L':>6} | {'Fy_R':>6} | {'Fz_R':>6}")
        print("="*85)

    def get_next_parameters(self):
        return self.optimizer.suggest()

    def register_result(self, parameters, reward):
        self.iteration += 1
        try:
            self.optimizer.register(params=parameters, target=reward)
            
            p = parameters
            if self.jump_type == 'forward':
                print(f"{self.iteration:>4} | {reward:>8.4f} | {p['f0_front']:>5.2f} | {p['f0_back']:>5.2f} | {p['Fx_front']:>6.1f} | {p['Fz_front']:>6.1f} | {p['Fx_back']:>6.1f} | {p['Fz_back']:>6.1f}")
            elif self.jump_type == 'twist':
                print(f"{self.iteration:>4} | {reward:>8.4f} | {p['f0_front']:>5.2f} | {p['f0_back']:>5.2f} | {p['Fy_front']:>6.1f} | {p['Fz_front']:>6.1f} | {p['Fy_back']:>6.1f} | {p['Fz_back']:>6.1f}")
            elif self.jump_type == 'lateral':
                print(f"{self.iteration:>4} | {reward:>8.4f} | {p['f0_left']:>5.2f} | {p['f0_right']:>5.2f} | {p['Fy_left']:>6.1f} | {p['Fz_left']:>6.1f} | {p['Fy_right']:>6.1f} | {p['Fz_right']:>6.1f}")
            
        except Exception as e:
            print(f"[Optimizer] Failed to register result: {e}")

    def get_best_jump(self):
        try:
            best = self.optimizer.max
            print("\n" + "*"*50)
            print(" BEST JUMP FOUND:")
            
            if self.jump_type == 'twist':
                print(f" Score: {best['target']:.4f} radians")
            else:
                print(f" Score: {best['target']:.4f} meters")
                
            for k, v in best['params'].items():
                print(f" {k:>10}: {v:.2f}")
            print("*"*50 + "\n")
            return best
        except ValueError:
            return {"target": 0.0, "params": "No successful jumps yet."}