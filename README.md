# Design and Operation of a UAVs Swarm for Optimal Performance

This repository contains the implementation of a decentralized coordination and resilient control architecture for Unmanned Aerial Vehicle (UAV) swarms. Developed as a Senior Design Project at Qassim University, this system addresses the fragility of conventional swarm formations in complex, dynamic environments.

# Overview
Standard swarm formations often struggle with collision avoidance in cluttered spaces, leading to "rigid" behaviors that result in mid-air impacts or swarm separation. Our solution integrates a Leader-Follower strategy with a predictive Artificial Potential Fields (APF) collision avoidance algorithm.


# Key Features
Predictive Collision Avoidance: Uses an Artificial Potential Fields (APF) algorithm that calculates real-time "urgency" metrics to adjust flight paths and prevent inter-agent collisions.

Elastic Formation Recovery: Unlike rigid control systems, this architecture allows agents to temporarily deviate for safety and then autonomously return to their assigned formation slots.

Decentralized Coordination: Each follower drone estimates the leader's motion and maintains its own formation offset using local state estimates.

High-Fidelity Simulation: Built and validated using Gazebo Harmonic with synchronized physics for Holybro X500 v2 airframes.

# Technical Stack
Operating System: Ubuntu 24.04 LTS

Middleware: ROS 2 (Jazzy Jalisco)

Flight Stack: PX4 Autopilot (v1.16)

Communication: Micro XRCE-DDS Agent and Client

Simulator: Gazebo Harmonic

# System Architecture
The control architecture unifies global reference frames through a local Cartesian NED frame based on a Flat-Earth Approximation. Follower velocity is dynamically scaled to reduce formation lag while prioritizing safety-critical repulsion forces.

# Getting Started 

Prerequisites

  1-Install ROS 2 Jazzy.

  2-Set up PX4 Autopilot v1.16.

  3-Install QGroundControl.

  4-Configure the Micro XRCE-DDS bridge.
  
# Installation & Launch

# Clone the repository
<pre>git clone https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git
cd ws_offboard_control

# Build the workspace
colcon build

# Launch the simulation (Drone 1 / Gazebo Server)
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
  (Note: To launch additional drones, open new terminals and use the PX4_GZ_STANDALONE=1 flag with unique IDs as detailed in the setup guide.)
</pre>

# Project Authors

Hamad Alsaleem

Sultan Alharbi 

Mohammed Altuwayjiri 

Omar Alharbi 

Supervisor: Dr. Mohammed Alfayizi, Qassim University
