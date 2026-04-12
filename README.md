# Design and Operation of a UAVs Swarm for Optimal Performance

[![ROS 2 - Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/index.html)
[![PX4 - v1.16](https://img.shields.io/badge/PX4-v1.16-012C6E?logo=px4&logoColor=white)](https://px4.io/)
[![Gazebo - Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License - MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## 1. Overview

This repository contains the implementation of a decentralized coordination and resilient control architecture for Unmanned Aerial Vehicle (UAV) swarms. Developed as a Senior Design Project at Qassim University, this system addresses the fragility of conventional swarm formations in complex, dynamic environments.

Standard swarm formations often struggle with collision avoidance in cluttered spaces, leading to "rigid" behaviors that result in mid-air impacts or swarm separation. Our solution integrates a Leader-Follower strategy with a predictive Artificial Potential Fields (APF) collision avoidance algorithm to ensure physical survivability without sacrificing mission objectives.

## 2. Key Features

* **Predictive Collision Avoidance:** Uses a modified APF algorithm that calculates real-time spatial and temporal "urgency" metrics to adjust flight paths and prevent inter-agent collisions.
* **Elastic Formation Recovery:** Unlike rigid control systems, this architecture allows agents to temporarily deviate from their assigned formation slots for safety (up to 7.5 meters) and then autonomously return.
* **Decentralized Coordination:** Each follower drone estimates the leader's motion and maintains its own formation offset using local state estimates.
* **High-Fidelity Simulation:** Built and validated using Gazebo Harmonic with synchronized physics for 5 Holybro X500 v2 airframes equipped with 2D LiDAR.

## 3. System Architecture

The control architecture unifies global reference frames through a local Cartesian NED frame based on a Flat-Earth Approximation. Follower velocity is dynamically scaled (using a constant feedforward multiplier) to reduce formation lag while prioritizing safety-critical repulsion forces during high-stress convergence events, such as 90-degree cornering.

## 4. Tech Stack

* **Operating System:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2 (Jazzy Jalisco)
* **Flight Stack:** PX4 Autopilot (v1.16)
* **Communication:** Micro XRCE-DDS Agent and Client
* **Simulator:** Gazebo Harmonic

---

## 5. Getting Started

### Prerequisites

Ensure you have the following installed and configured on your system:
1. ROS 2 Jazzy
2. PX4 Autopilot (v1.16)
3. QGroundControl (Optional: For monitoring only)
4. Micro XRCE-DDS bridge

### Clone the Repository

```bash
git clone https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git
cd ws_offboard_control
```

---

## 6. How to Start the Simulation

To launch the full 5-drone swarm, you will need to open multiple terminal tabs. We have provided a shell script to automate the drone launching process.

### Step 1: Build the Workspace
Always build the workspace if you have made changes to the code.
```bash
# Terminal 1
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_ros_com
source install/local_setup.bash
```

### Step 2: Start the MicroXRCE Agent
This bridge allows ROS 2 to communicate with PX4.
```bash
# Terminal 2
MicroXRCEAgent udp4 -p 8888
```

### Step 3: Launch the Drones & Simulator
Instead of launching 5 instances manually, use the provided launch script.
```bash
# Terminal 3
cd ~/ws_offboard_control
./launch_drones.sh
```

### Step 4: Launch the Swarm Bridge & RViz
```bash
# Terminal 4 (LiDAR/Swarm Bridge)
cd ~/ws_offboard_control
source install/local_setup.bash
ros2 run px4_ros_com swarm_bridge.py --num-drones 5 --world default

# Terminal 5 (RViz Visualization)
cd ~/ws_offboard_control
source install/local_setup.bash
ros2 launch px4_ros_com swarm_viz.launch.py
```

### Step 5: Start Offboard Control & Path
Initialize the control node, then send the trajectory path (e.g., a square path).
```bash
# Terminal 6 (Offboard Control Node)
cd ~/ws_offboard_control
source install/local_setup.bash
ros2 run px4_ros_com multi_vehicle_offboard_control -- --num_drones 5

# Terminal 7 (Send Path Command)
cd ~/ws_offboard_control
source install/local_setup.bash
python3 src/px4_ros_com/src/examples/offboard_py/path_controller.py --path square 
```

---

## 7. Expected Results & Visualizations

> **Note:** *[Insert GIF/Video of the simulation running in Gazebo here]*

After running a simulation, flight data is logged into CSV files. You can generate 2D and 3D trajectory graphs to analyze the formation error and separation distances.

To visualize the path from a recent flight:
```bash
cd ~/ws_offboard_control
source install/local_setup.bash

# Replace the CSV filename with your latest log file
python3 src/px4_ros_com/src/examples/offboard_py/visualize_path.py plots/square_2.0ms_20251202_120103.csv/square_2.0ms_20251202_120103.csv
```

> **Note:** *[Insert Image of the 3D plotted graph here]*

---

## 8. Turning Off Collision Avoidance for Testing

To establish a baseline or test strict geometric tracking without safety overrides, you can disable the APF collision avoidance module. 

> **Developer Note:** *[Explain briefly here how the user disables it in your code. E.g., "Change the ENABLE_APF boolean to False in multi_vehicle_offboard_control.py before building."]*

**Warning:** Disabling collision avoidance during dynamic maneuvers (like the square path) will result in catastrophic 0-meter mid-air collisions as the agents attempt to perfectly track overlapping geometric slots.

---

## 9. Useful Commands & Troubleshooting

**Kill All Simulation Processes:**
If the simulation crashes or hangs, use this command to cleanly wipe all background processes before restarting:
```bash
tmux kill-session -t drones; killall -9 px4 gzserver gzclient ruby gz MicroXRCEAgent; pkill -9 -f "PX4_SYS_AUTOSTART" 
```

**Clean Build:**
If you encounter caching issues, wipe the build and install folders:
```bash
cd ~/ws_offboard_control
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
```

---

## Project Authors

* **Hamad Alsaleem**
* **Sultan Alharbi**
* **Mohammed Altuwayjiri**
* **Omar Alharbi**

**Supervisor:** Dr. Mohammed Alfayiz, Department of Engineering, Qassim University

### Citation
If you use this code or simulation architecture in your research, please cite our paper:
> Alharbi, S.; Altuwayjiri, M.; Alharbi, O.; Alsaleem, H.; Alfayiz, M. *Decentralized Predictive-Repulsive Formation Control for UAV Swarms in Constrained Environments.* Appl. Sci. 2026.
