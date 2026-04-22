import time
import numpy as np
import sys
import select
import config
import socket
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
        
        # Initialize the array container (values will be set dynamically)
        self.nominal_p_world = []
        self.set_nominal_height(-0.2) # Default to crouch on startup

    def set_nominal_height(self, z_height):
        """Updates the target resting height of the robot."""
        self.nominal_p_world = []
        for leg in range(4):
            side_sign = 1.0 if leg in [1, 3] else -1.0
            p_leg = np.array([0.0, side_sign * 0.0955, z_height])
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


    def main_loop(self, f0, f1, Fx, Fy, Fz):
        q, dq, roll, pitch, _ = self.get_robot_state()
        
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
            
            # --- CRITICAL: Prevents MuJoCo physics explosions! ---
            #tau = np.clip(tau, -35.0, 35.0)
            
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
        Fx, Fy, Fz = params['Fx'], params.get('Fy', 0.0), params['Fz'] 
        
        jumping = False
        landing = False
        landing_counter = 0
        x_init = 0.0
        
        stand_start_time = time.time()
        
        # PREPARE: Set to crouched position before jumping
        self.set_nominal_height(-0.2)
        
        while True:
            if not jumping and not landing:
                self.main_loop(f0, f1, 0.0, 0.0, 0.0)
                
                # Automatically jump after 2 seconds
                if time.time() - stand_start_time > 2.0:
                    self.cpg.theta = np.pi 
                    self.estimated_vx = 0.0
                    self.estimated_x = 0.0
                    self.last_time = time.time()
                    _, _, _, _, x_init = self.get_robot_state()
                    jumping = True
                    # (Removed the instant snap to -0.32 from here)
                    
            elif jumping:
                # ---> THE FIX: Smoothly deploy legs during the thrust <---
                # Read the current target height of the first leg
                current_z = self.nominal_p_world[0][2] 
                
                # If we haven't reached full extension yet, lower it by 2mm
                if current_z > -0.32:
                    self.set_nominal_height(current_z - 0.002) 
                # ---------------------------------------------------------
                
                done = self.main_loop(f0, f1, Fx, Fy, Fz)
                if done:
                    jumping = False
                    landing = True
                    
            elif landing:
                # Continue smooth deployment just in case the jump was fast
                current_z = self.nominal_p_world[0][2] 
                if current_z > -0.32:
                    self.set_nominal_height(current_z - 0.002)
                    
                self.main_loop(f0, f1, 0.0, 0.0, 0.0)
                landing_counter += 1
                if landing_counter >= 2000: 
                    break
                    
            # -------------------------------------------------------------
            # THE GUARD: Fall Detection
            # -------------------------------------------------------------
            _, _, roll, pitch, _ = self.get_robot_state()
            
            if abs(roll) > 1.0 or abs(pitch) > 1.6:
                for i in range(12):
                    self.low_cmd.motor_cmd[i].tau = 0.0
                    self.low_cmd.motor_cmd[i].kp = 0.0
                    self.low_cmd.motor_cmd[i].kd = 0.0
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.cmd_pub.Write(self.low_cmd)
                
                return 0.0 # Return 0 for a crash

            time.sleep(0.001)

        _, _, _, _, x_final = self.get_robot_state()
        return x_final - x_init

    def run_optimization(self, trials=50):
        # Setup the UDP sender
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        for i in range(trials):
            # 1. COMMAND THE SIMULATOR TO RESET
            sock.sendto(b"RESET", ('localhost', 9876))
            
            # 2. Give the physics engine 2.0 seconds to drop the robot and let it settle
            time.sleep(2.0) 
            
            params = self.optimizer.get_next_parameters()
            score = self.execute_single_jump(params)
            self.optimizer.register_result(params, score)
            
        print("\n--- Optimization Complete ---")

if __name__ == '__main__':
    experiment = FrogJumpExperiment()
    experiment.run_optimization(trials=50)