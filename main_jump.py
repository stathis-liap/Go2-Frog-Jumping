import time
import numpy as np
import sys
import select
import config
from force_generator import JumpForceGenerator
from controllers import LegController, calculate_vmc_forces
from gp_optimizer import JumpOptimizer
from kinematics import Go2Kinematics

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.utils.crc import CRC 

class FrogJumpExperiment:
    def __init__(self):
        ChannelFactoryInitialize(1, "lo")
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        
        self.low_state = unitree_go_msg_dds__LowState_()
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self._low_state_handler, 10)

        self.sport_state = None 
        self.sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sport_sub.Init(self._sport_state_handler, 10)
        
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.crc = CRC()
        
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        
        self.optimizer = JumpOptimizer()
        self.cpg = JumpForceGenerator(dt=0.001) 
        self.leg_controller = LegController()
        self.kinematics = Go2Kinematics() 

        self.estimated_vx = 0.0
        self.estimated_x = 0.0
        self.last_time = time.time()
        
        self.nominal_p_world = []
        for leg in range(4):
            side_sign = 1.0 if leg in [1, 3] else -1.0
            p_leg = np.array([0.0, side_sign * 0.0955, -0.22])
            self.nominal_p_world.append(p_leg)

    def _low_state_handler(self, msg: LowState_):
        self.low_state = msg

    def _sport_state_handler(self, msg: SportModeState_):
        self.sport_state = msg

    def get_robot_state(self):
        q, dq = np.zeros(12), np.zeros(12)
        for i in range(12):
            q[i] = self.low_state.motor_state[i].q
            dq[i] = self.low_state.motor_state[i].dq
            
        quat = self.low_state.imu_state.quaternion
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        
        if self.sport_state is not None:
            true_x_position = self.sport_state.position[0]
        else:
            true_x_position = 0.0 
            
        return q, dq, roll, pitch, true_x_position


    def run_unified_paper_loop(self, f0, f1, Fx, Fy, Fz):
        q, dq, roll, pitch, _ = self.get_robot_state()
        
        # print(f"IMU Alive -> Pitch: {pitch:.3f} | Roll: {roll:.3f}")
        
        from controllers import get_rotation_matrix
        R = get_rotation_matrix(roll, pitch)
        P_base = np.array([[1, 1, -1, -1], [-1, 1, -1, 1], [0, 0, 0, 0]])
        
        F_foot_per_leg, _, done = self.cpg.step(f0, f1, Fx, Fy, Fz)
        F_vmc_body = calculate_vmc_forces(R, P_base)
        
        for leg in range(4):
            leg_q, leg_dq = q[leg*3:(leg+1)*3], dq[leg*3:(leg+1)*3]
            p_curr, J_real = self.kinematics.compute_leg_kinematics(leg_q, leg)
            
            p_desired_body = R.T @ self.nominal_p_world[leg]
            
            tau = self.leg_controller.compute_total_torque(
                J=J_real, 
                F_jump=F_foot_per_leg, 
                p_current=p_curr, 
                p_desired=p_desired_body,  
                v_current=J_real @ leg_dq, 
                dq_current=leg_dq, 
                F_vmc_leg_body=F_vmc_body[:, leg], 
                R=R
            )
            
            # tau = np.clip(tau, -35.0, 35.0)
            
            for j in range(3):
                idx = leg*3 + j
                self.low_cmd.motor_cmd[idx].mode = 0x01
                self.low_cmd.motor_cmd[idx].q = 0.0 
                self.low_cmd.motor_cmd[idx].kp = 0.0  
                self.low_cmd.motor_cmd[idx].kd = 0.0  
                self.low_cmd.motor_cmd[idx].tau = float(tau[j])
                
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_pub.Write(self.low_cmd)
        return done

    def execute_single_jump(self, params):
        f0, f1 = params['f0'], 1.0
        Fx, Fy, Fz = params['Fx'], 0.0, params['Fz'] 

        # print(">>> STANDING ON PURE MATH... Press ENTER to JUMP <<<")
        
        jumping = False
        landing = False
        landing_counter = 0
        x_init = 0.0
        
        while True:
            if not jumping and not landing:
                self.run_unified_paper_loop(f0, f1, 0.0, 0.0, 0.0)
                
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    
                    # print(f"--- Jump Impulse [Fx:{Fx:.1f}, Fz:{Fz:.1f}] ---")
                    self.cpg.theta = np.pi 
                    
                    self.estimated_vx = 0.0
                    self.estimated_x = 0.0
                    self.last_time = time.time()
                    _, _, _, _, x_init = self.get_robot_state()
                    jumping = True
                    
            elif jumping:
                done = self.run_unified_paper_loop(f0, f1, Fx, Fy, Fz)
                if done:
                    # print("Impulse complete. Catching landing...")
                    jumping = False
                    landing = True
                    
            elif landing:
                self.run_unified_paper_loop(f0, f1, 0.0, 0.0, 0.0)
                landing_counter += 1
                if landing_counter >= 1500:
                    break
                    
            # THE GUARD: Fall Detection
            _, _, roll, pitch, _ = self.get_robot_state()
            
            if abs(roll) > 1.0 or abs(pitch) > 1.0:
                # print(f"!!! CRASH DETECTED !!! (Roll: {roll:.2f}, Pitch: {pitch:.2f})")
                
                for i in range(12):
                    self.low_cmd.motor_cmd[i].tau = 0.0
                    self.low_cmd.motor_cmd[i].kp = 0.0
                    self.low_cmd.motor_cmd[i].kd = 0.0
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.cmd_pub.Write(self.low_cmd)
                
                # Zero the objective function
                return 0.0
            # -------------------------------------------------------------

            time.sleep(0.001)

        _, _, _, _, x_final = self.get_robot_state()
        return x_final - x_init

    def run_optimization(self, trials=10):
        # print("Starting Quadruped-Frog Online Optimization...")
        for i in range(trials):
            # print(f"\n[Trial {i+1}/10] Reset MuJoCo and wait...")
            input()
            params = self.optimizer.get_next_parameters()
            score = self.execute_single_jump(params)
            self.optimizer.register_result(params, score)
        print("\n--- Optimization Complete ---")
        print(f"\n Best results {self.optimizer.get_best_jump()}")

if __name__ == '__main__':
    experiment = FrogJumpExperiment()
    experiment.run_optimization(trials=10)