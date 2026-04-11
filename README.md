# Go2-Frog-Jumping

[![University](https://img.shields.io/badge/University-Patras-blue)](#) [![Club](https://img.shields.io/badge/Lab-Embodied_Intelligence_Club-orange)](#) [![Robot](https://img.shields.io/badge/Hardware-Unitree_Go2-lightgrey)](#)

Developed by the **Embodied Intelligence Club at the University of Patras**, this repository implements a continuous, pure-math jumping controller on the 15kg Unitree Go2 quadruped.

### 📄 Based on Academic Research
This project is a direct hardware port and heavy adaptation of the state-of-the-art continuous control architecture proposed in:
> **"[Quadruped-Frog: Rapid Online Optimization of Continuous Quadruped Jumping]"** by [Guillaume Bellegarda, Milad Shafiee, Merih Ekin Özberk, Auke Ijspeert]. 
> *Read the full paper here: [Link to ArXiv: https://arxiv.org/abs/2403.06954]*

Our implementation successfully scales the paper's lightweight robot theorems to a heavyweight, commercial quadruped by introducing dynamic frame translations, gravity feed-forward logic, and VMC mass-scaling.

### 🎯 The Goal
To achieve explosive, perfectly stabilized forward jumping and safe shock-absorption on a commercial quadruped by bypassing the rigid hardware firmware and utilizing pure mathematical torque control. 

### ⚙️ How it is Achieved
Instead of using discrete state-machines (where the robot sharply switches between "stand," "jump," and "land" code), all forces are calculated and summed continuously at 1kHz as proposed by the foundational paper.

* **Suspension & Foot Tracking (Cartesian PD):** The robot abandons traditional joint-angle locking. Instead, it uses a Cartesian PD Impedance controller mathematically anchored to the world frame. This acts as a virtual shock-absorber, allowing the robot to seamlessly catch its own 15kg falling weight without shattering its gearboxes.
* **Attitude Stabilization (VMC):** A heavily scaled Virtual Model Controller tracks the robot's IMU. When the massive 1,400N jump thrust inevitably causes the robot's nose to pitch forward, the VMC injects counter-torque into the front legs to keep the torso perfectly level mid-air.
* **Explosive Thrust (CPG):** A parameterized Central Pattern Generator injects raw feed-forward force ($F_x$, $F_z$) mapped directly through the Jacobian Transpose to launch the robot.
* **Autonomous Learning (Bayesian Optimization):** The system integrates with the Go2's internal SLAM / Extended Kalman Filter (EKF) to precisely measure the distance of every jump. A Gaussian Process iteratively hunts for the perfect thrust and timing parameters, autonomously mapping the robot's unique hardware limits.

### 🚀 Key Features
* **Zero-Limp Architecture:** A seamless 1kHz control loop that entirely bypasses the Go2's hardware watchdog timer.
* **Dynamic Frame Translation:** Continuous rotational math to ensure the Body Frame and World Frame stay perfectly synced, preventing "rigid cast" joint lockups.
* **Early Crash Termination:** Automated guardrails that detect catastrophic pitch/roll failures (> 57°) and instantly kill motor torques to protect the hardware while penalizing the optimizer.

---
*Built with 💻 and ☕ by the Embodied Intelligence Club.*
