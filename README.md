# Quadruped Frog Jump: Continuous Cartesian Flight for the Unitree Go2

[![University](https://img.shields.io/badge/University-Patras-blue)](#) [![Club](https://img.shields.io/badge/Lab-Embodied_Intelligence_Club-orange)](#) [![Robot](https://img.shields.io/badge/Hardware-Unitree_Go2-lightgrey)](#)

Developed by the **Embodied Intelligence Club at the University of Patras**, this repository implements a continuous, pure-math jumping controller on the 15kg Unitree Go2 quadruped.

### 📄 Based on Academic Research
This project is a direct hardware port and heavy adaptation of the state-of-the-art continuous control architecture proposed in:
> **"Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping"** by Guillaume Bellegarda, Milad Shafiee, Merih Ekin Özberk, Auke Ijspeert. 
> *Read the full paper here: https://arxiv.org/abs/2403.06954*

Our implementation successfully scales the paper's lightweight robot theorems to a heavyweight, commercial quadruped by introducing dynamic frame translations, gravity feed-forward logic, and VMC mass-scaling.

### 🎯 The Goal
To achieve explosive, perfectly stabilized forward jumping and safe shock-absorption on a commercial quadruped by bypassing the rigid hardware firmware and utilizing pure mathematical torque control. 

### ⚙️ How it is Achieved
Instead of using discrete state-machines (where the robot sharply switches between "stand," "jump," and "land" code), all forces are calculated and summed continuously at 1kHz as proposed by the foundational paper.

* **Suspension & Foot Tracking (Cartesian PD):** The robot abandons traditional joint-angle locking. Instead, it uses a Cartesian PD Impedance controller mathematically anchored to the world frame. This acts as a virtual shock-absorber, allowing the robot to seamlessly catch its own 15kg falling weight without shattering its gearboxes.
* **Attitude Stabilization (VMC):** A heavily scaled Virtual Model Controller tracks the robot's IMU. When the massive 1,400N jump thrust inevitably causes the robot's nose to pitch forward, the VMC injects counter-torque into the front legs to keep the torso perfectly level mid-air.
* **Explosive Thrust (CPG):** A parameterized Central Pattern Generator injects raw feed-forward force ($F_x$, $F_z$) mapped directly through the Jacobian Transpose to launch the robot.
* **Autonomous Learning (Bayesian Optimization):** The system integrates with the Go2's internal SLAM / Extended Kalman Filter (EKF) to precisely measure the distance of every jump. A Gaussian Process iteratively hunts for the perfect thrust and timing parameters, autonomously mapping the robot's unique hardware limits.

---

## 🛠️ Getting Started (Installation & Setup)

This project requires a specific dual-terminal setup using the Unitree Python SDK and the Unitree MuJoCo simulator. 

### 1. Create a Virtual Environment
It is highly recommended to isolate the dependencies for this project. 
```bash
# Create the workspace directory
mkdir -p ~/Quadruped_Project
cd ~/Quadruped_Project

# Create and activate a Python virtual environment
python3 -m venv go2_env
source go2_env/bin/activate
```
### 2. Install the Unitree Python SDK (unitree_sdk2py)
This allows your Python scripts to communicate with the Go2 (or the simulator) over the internal DDS network.

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2py.git
cd unitree_sdk2py
pip install -e .
cd ..
```

### 3. Install the Unitree MuJoCo Simulator
This is the official physics simulator that mimics the Go2's hardware and DDS network.
```bash
cd ~/Quadruped_Project

# Install the MuJoCo engine
pip install mujoco

# Clone the Unitree simulation repository
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### 4. Install the Bayesian Optimization Library
```bash
pip install git+https://github.com/bayesian-optimization/BayesianOptimization.git
```
### 5. Clone this Repository
```bash
git clone https://github.com/stathis-liap/Go2-Frog-Jumping.git frog_jump
cd frog_jump
pip install numpy
```

---

### 🚀 Running the Optimization
To run the experiment, you need to open two separate terminal windows. Ensure your virtual environment is activated in both!

Terminal 1: Start the Physics Simulator
This spins up MuJoCo and the simulated DDS network.
```bash
source ~/Quadruped_Project/go2_env/bin/activate
cd ~/Quadruped_Project/unitree_mujoco/simulate_python
python unitree_mujoco.py
```

Terminal 2: Run the Jumping Controller
Wait for the simulator to fully load, then launch the optimizer.
```bash
source ~/Quadruped_Project/go2_env/bin/activate
cd ~/Quadruped_Project/frog_jump
python main_jump.py
```


