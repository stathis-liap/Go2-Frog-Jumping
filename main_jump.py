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
    def __init__(self, jump_type='forward'):
        self.jump_type = jump_type  
        
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
        
        self.optimizer = JumpOptimizer(jump_type=self.jump_type)
        self.cpg = JumpForceGenerator(dt=0.001) 
        self.leg_controller = LegController()
        self.kinematics = Go2Kinematics() 

        self.estimated_vx = 0.0
        self.estimated_x = 0.0
        self.last_time = time.time()
        
        self.nominal_p_body = []
        self.set_nominal_height(-0.2) 

    def set_nominal_height(self, z_height, y_offset=0.0):
        self.nominal_p_body = []
        for leg in range(4):
            side_sign = 1.0 if leg in [1, 3] else -1.0
            p_leg = np.array([0.0, side_sign * 0.0955 - y_offset, z_height])
            self.nominal_p_body.append(p_leg)

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

        roll  = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
        sinp  = 2.0 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        yaw   = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
        
        if self.sport_state is not None:
            true_x_position = self.sport_state.position[0]
            true_y_position = self.sport_state.position[1]
        else:
            true_x_position = 0.0
            true_y_position = 0.0

        return q, dq, roll, pitch, yaw, true_x_position, true_y_position

    def main_loop(self, f0, f1, Fx_arr, Fy_arr, Fz_arr, phase='stance'):
        q, dq, roll, pitch, _, _, _ = self.get_robot_state()

        from controllers import get_rotation_matrix
        R = get_rotation_matrix(roll, pitch)
        P_base = np.array([[1, 1, -1, -1], [-1, 1, -1, 1], [0, 0, 0, 0]])

        F_per_leg, _, done = self.cpg.step(f0, f1, Fx_arr, Fy_arr, Fz_arr)
        F_vmc_body = calculate_vmc_forces(R, P_base)

        for leg in range(4):
            leg_q, leg_dq = q[leg*3:(leg+1)*3], dq[leg*3:(leg+1)*3]
            p_curr, J_real = self.kinematics.compute_leg_kinematics(leg_q, leg)
            p_desired_body = self.nominal_p_body[leg]

            tau = self.leg_controller.compute_total_torque(
                J=J_real,
                F_jump=F_per_leg[:, leg], 
                p_current=p_curr,
                p_desired=p_desired_body,
                v_current=J_real @ leg_dq,
                dq_current=leg_dq,
                F_vmc_leg_body=F_vmc_body[:, leg],
                R=R
            )

            tau = np.clip(tau, -35.0, 35.0)

            for j in range(3):
                idx = leg*3 + j
                self.low_cmd.motor_cmd[idx].mode = 0x01
                self.low_cmd.motor_cmd[idx].q   = 0.0
                self.low_cmd.motor_cmd[idx].kp  = 0.0
                self.low_cmd.motor_cmd[idx].kd  = 0.0
                self.low_cmd.motor_cmd[idx].tau = float(tau[j])

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.cmd_pub.Write(self.low_cmd)
        return done

    def execute_single_jump(self, params):
        f1 = 1.0 # Fixed flight phase frequency
        
        f0_arr = np.zeros(4)
        Fx_arr = np.zeros(4)
        Fy_arr = np.zeros(4)
        Fz_arr = np.zeros(4)
        
        # Leg Index: 0=FR, 1=FL, 2=RR, 3=RL
        if self.jump_type == 'forward':
            f0_arr[0] = f0_arr[1] = params['f0_front']
            f0_arr[2] = f0_arr[3] = params['f0_back']
            Fx_arr[0] = Fx_arr[1] = params['Fx_front']
            Fx_arr[2] = Fx_arr[3] = params['Fx_back']
            Fz_arr[0] = Fz_arr[1] = params['Fz_front']
            Fz_arr[2] = Fz_arr[3] = params['Fz_back']
            # Fy_arr remains completely zeroed
            
        elif self.jump_type == 'twist':
            f0_arr[0] = f0_arr[1] = params['f0_front']
            f0_arr[2] = f0_arr[3] = params['f0_back']
            Fz_arr[0] = Fz_arr[1] = params['Fz_front']
            Fz_arr[2] = Fz_arr[3] = params['Fz_back']
            
            # Invert left legs to push in the same world direction
            Fy_arr[0] = params['Fy_front']
            Fy_arr[1] = -params['Fy_front']
            Fy_arr[2] = params['Fy_back']
            Fy_arr[3] = -params['Fy_back']
            # Fx_arr remains completely zeroed
            
        elif self.jump_type == 'lateral':
            # Left Legs: 1, 3. Right Legs: 0, 2
            f0_arr[1] = f0_arr[3] = params['f0_left']
            f0_arr[0] = f0_arr[2] = params['f0_right']
            Fz_arr[1] = Fz_arr[3] = params['Fz_left']
            Fz_arr[0] = Fz_arr[2] = params['Fz_right']
            
            # Invert left legs to push in the same world direction
            Fy_arr[0] = Fy_arr[2] = params['Fy_right']
            Fy_arr[1] = Fy_arr[3] = -params['Fy_left']
            # Fx_arr remains completely zeroed

        jumping = False
        landing = False
        landing_counter = 0

        x_init, y_init, yaw_init = 0.0, 0.0, 0.0
        
        # Start in a deep crouch (-0.2m) 
        self.set_nominal_height(-0.24)
        stand_start_time = time.time()
        
        zero_arr = np.zeros(4)

        while True:
            if not jumping and not landing:
                # Use zero forces for stance
                self.main_loop(f0_arr, f1, zero_arr, zero_arr, zero_arr)

                if time.time() - stand_start_time > 2.0:
                    self.cpg.theta = np.full(4, np.pi) 
                    self.estimated_vx = 0.0
                    self.estimated_x = 0.0
                    self.last_time = time.time()
                    _, _, _, _, yaw_init, x_init, y_init = self.get_robot_state()
                    jumping = True

            elif jumping:
                done = self.main_loop(f0_arr, f1, Fx_arr, Fy_arr, Fz_arr)
                if done:
                    jumping = False
                    landing = True

            elif landing:
                current_z = self.nominal_p_body[0][2]
                if current_z > -0.30:
                    self.set_nominal_height(current_z - 0.0005) 
                    
                self.main_loop(f0_arr, f1, zero_arr, zero_arr, zero_arr)
                landing_counter += 1
                if landing_counter >= 2000:
                    break

            _, _, roll, pitch, _, _, _ = self.get_robot_state()

            # Crash guard
            if abs(roll) > 1.8 or abs(pitch) > 1.8:
                for i in range(12):
                    self.low_cmd.motor_cmd[i].tau = 0.0
                    self.low_cmd.motor_cmd[i].kp  = 0.0
                    self.low_cmd.motor_cmd[i].kd  = 0.0
                self.low_cmd.crc = self.crc.Crc(self.low_cmd)
                self.cmd_pub.Write(self.low_cmd)
                return 0.0

            time.sleep(0.001)

        _, _, _, _, yaw_final, x_final, y_final= self.get_robot_state()

        # Score calculations with absolute values to prevent punishing "wrong direction" jumps
        if self.jump_type == 'forward':
            return abs(x_final - x_init)                    
        elif self.jump_type == 'lateral':
            return abs(y_final - y_init)                    
        elif self.jump_type == 'twist':
            return abs(yaw_final - yaw_init)                
        else:
            return abs(x_final - x_init)

    def run_optimization(self, trials=50):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        for i in range(trials):
            sock.sendto(b"RESET", ('localhost', 9876))
            time.sleep(2.0) 
            
            params = self.optimizer.get_next_parameters()
            score = self.execute_single_jump(params)
            self.optimizer.register_result(params, score)
            
        print("\n--- Optimization Complete ---")
        self.optimizer.get_best_jump()


if __name__ == '__main__':
    print("\n" + "="*40)
    print(" QUADRUPED FROG JUMP OPTIMIZER")
    print("="*40)
    print("Select the type of jump to optimize:")
    print("1. Forward Jumping (Max X)")
    print("2. Lateral Jumping (Max Y)")
    print("3. Twist Jumping (Max Yaw)")
    
    choice = input("\nEnter choice (1/2/3): ")
    
    jump_map = {'1': 'forward', '2': 'lateral', '3': 'twist'}
    selected_jump = jump_map.get(choice, 'forward') 
    
    print(f"\n[+] Optimizing for: {selected_jump.upper()} JUMP\n")
    
    experiment = FrogJumpExperiment(jump_type=selected_jump)
    experiment.run_optimization(trials=50)