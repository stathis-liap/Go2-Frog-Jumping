from bayes_opt import BayesianOptimization
from bayes_opt.acquisition import UpperConfidenceBound
import config

class JumpOptimizer:
    def __init__(self, random_state=42):
        """
        Initializes the Gaussian Process Bayesian Optimizer (V2 API).
        Custom logging implemented to match the paper's optimization graphs.
        """
        self.pbounds = {
            'f0': config.BOUNDS_F0,
            'Fx': config.BOUNDS_FX,
            'Fy': config.BOUNDS_FY,
            'Fz': config.BOUNDS_FZ
        }

        # kappa=2.5 balances exploration vs exploitation
        self.acq = UpperConfidenceBound(kappa=2.5)

        # Set verbose=0 to turn off the ugly default bayes_opt dictionary logs
        self.optimizer = BayesianOptimization(
            f=None, 
            pbounds=self.pbounds,
            acquisition_function=self.acq,
            verbose=0, 
            random_state=random_state
        )
        
        self.iteration = 0
        
        print("\n" + "="*70)
        print(f"{'Iter':>5} | {'Objective (m)':>13} | {'f (Hz)':>8} | {'Fx (N)':>8} | {'Fy (N)':>8} | {'Fz (N)':>8}")
        print("="*70)

    def get_next_parameters(self):
        return self.optimizer.suggest()

    def register_result(self, parameters, reward):
        self.iteration += 1
        try:
            self.optimizer.register(
                params=parameters,
                target=reward
            )
            
            f_val = parameters['f0']
            fx_val = parameters['Fx']
            fy_val = parameters['Fy']
            fz_val = parameters['Fz']
            
            print(f"{self.iteration:>5} | {reward:>13.4f} | {f_val:>8.3f} | {fx_val:>8.1f} | {fy_val:>8.1f} | {fz_val:>8.1f}")
            
        except Exception as e:
            print(f"[Optimizer] Failed to register result: {e}")

    def get_best_jump(self):
        try:
            return self.optimizer.max
        except ValueError:
            return {"target": 0.0, "params": "No successful jumps yet."}