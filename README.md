# Design and Operation of a UAVs Swarm for Optimal Performance

[![ROS 2 - Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/index.html)
[![PX4 - v1.16](https://img.shields.io/badge/PX4-v1.16-012C6E?logo=px4&logoColor=white)](https://docs.px4.io/main/en/)
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

The swarm utilizes a decentralized architecture that separates the complex swarm intelligence from the basic flight mechanics. All decision-making happens locally on each drone, preventing the communication bottlenecks you see in centralized ground-station setups.

The system is split into two primary layers:

* **High-Level Layer:** This handles the heavy lifting—trajectory prediction, formation math, and the APF collision avoidance logic.
* **Low-Level Layer:** This runs directly on the flight controller to manage vehicle attitude stabilization, motor mixing, and executing the requested velocity commands.

### Control Pipeline
Each individual drone runs a continuous 5-step loop:
1. Predict the leader's future position to compensate for communication latency.
2. Calculate its designated geometric formation offset.
3. Assess the real-time collision risk using the urgency metric.
4. Apply repulsive positional corrections if a threat is detected.
5. Dynamically scale velocity to efficiently close any resulting spatial gaps.

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
3. QGroundControl (Must be installed and running during simulation)
4. Micro XRCE-DDS bridge
5. **Terminator** (Terminal emulator required for the launch script. Install via: `sudo apt install terminator`)

### Clone the Repository

```bash
git clone [https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git](https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git)
cd ws_offboard_control
```

---

## 6. How to Start the Simulation

We have provided a shell script that utilizes `terminator` to automate the process of launching the Gazebo simulator, the Micro XRCE-DDS Agent, and multiple PX4 drone instances simultaneously.

### Step 1: Build the Workspace
Always build the workspace if you have made changes to the code.
```bash
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_ros_com
source install/local_setup.bash
```

### Step 2: Open QGroundControl
Before launching the swarm, open QGroundControl. The drones require an active connection to initialize and transition into flight modes.

### Step 3: Launch the Swarm and Simulator
Run the automated script. This will open a new `terminator` window running the Gazebo environment, the Micro XRCE-DDS Agent, and the 5 drone instances.
```bash
cd ~/ws_offboard_control
./launch_drones.sh
```

### Step 4: Start the Offboard Control Node
Open a new standard terminal to initialize the main swarm control node that handles the APF logic.
```bash
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run px4_ros_com multi_vehicle_offboard_control -- --num_drones 5
```

### Step 5: Send the Trajectory Command
Open one final terminal to command the swarm to fly a specific path (e.g., a square path).
```bash
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/px4_ros_com/src/examples/offboard_py/path_controller.py --path square 
```

---

## 7. Expected Results & Visualizations

https://github.com/user-attachments/assets/248425f8-e0a5-4c41-86d0-132112c3966c

> **Simulation Demo:** 5 UAVs executing a square trajectory with collision avoidance active.

After running a simulation, flight data is logged into CSV files. You can generate 2D and 3D trajectory graphs to analyze the formation error and separation distances.

To visualize the path from a recent flight:
```bash
cd ~/ws_offboard_control
source install/local_setup.bash

# Replace the CSV filename with your latest log file
python3 src/px4_ros_com/src/examples/offboard_py/visualize_path.py plots/square_2.0ms_20251202_120103.csv/square_2.0ms_20251202_120103.csv
```

<img width="407" height="356" alt="2D Trajectory Analysis" src="https://github.com/user-attachments/assets/c2958a2d-c892-4511-bc33-c012f209fc47" />

> **2D Trajectory:** Top-down view showing drones expanding their formation at corners to prevent collisions.

<img width="314" height="262" alt="3D Spatial Visualization" src="https://github.com/user-attachments/assets/68047585-658a-4573-9ff4-2b8dd0ed8310" />

> **3D Trajectory:** Drones shifting altitude during tight turns to maintain safe spacing.

---

## 8. Turning Off Collision Avoidance for Testing

To establish a baseline or test strict geometric tracking without safety overrides, you can manually disable the APF collision avoidance module in the source code.

1. Navigate to the main control script:
   `src/px4_ros_com/src/examples/offboard/multi_vehicle_offboard_control.cpp`
2. Open the file and change the collision avoidance flag from `true` to `false`.
3. Save the file and rebuild the workspace (`colcon build`) for the changes to take effect.

---

## 9. Useful Commands & Troubleshooting

**Kill All Simulation Processes:**
If the simulation crashes or hangs, use this command to cleanly wipe all background processes before restarting:
```bash
tmux kill-session -t drones; killall -9 px4 gzserver gzclient ruby gz MicroXRCEAgent terminator; pkill -9 -f "PX4_SYS_AUTOSTART" 
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

**Supervisor:** Dr. Mohammed Alfayizi, Department of Engineering, Qassim University
