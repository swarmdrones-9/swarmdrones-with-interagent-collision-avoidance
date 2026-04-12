# Decentralized Predictive-Repulsive Formation Control for UAV Swarms

[![ROS 2 - Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/index.html)
[![PX4 - v1.16](https://img.shields.io/badge/PX4-v1.16-012C6E?logo=px4&logoColor=white)](https://px4.io/)
[![Gazebo - Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![License - MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

This repository contains the implementation of a decentralized coordination and resilient control architecture for Unmanned Aerial Vehicle (UAV) swarms. Developed as a Senior Design Project at Qassim University, this system addresses the fragility of conventional swarm formations in complex, dynamic environments.

Standard swarm formations often struggle with collision avoidance in cluttered spaces, leading to "rigid" behaviors that result in mid-air impacts or swarm separation. Our solution integrates an existing Leader-Follower strategy with a predictive, urgency-based Artificial Potential Fields (APF) collision avoidance algorithm. 

## Simulation Results

> **Note to authors:** *Replace the placeholder image below with Figure 5 (3D View of Proposed Method) or an animated GIF of your SITL simulation in action!*

![Simulation Demo Placeholder](link-to-your-image-or-gif-here.png)

## Key Features

* **Predictive Collision Avoidance:** Utilizes a modified APF algorithm that calculates real-time spatial and temporal "urgency" metrics to preemptively adjust flight paths and prevent inter-agent collisions.
* **Elastic Formation Recovery:** Unlike rigid geometric control systems, this architecture prioritizes physical survivability, allowing agents to temporarily deviate for safety and then autonomously return to their assigned formation slots.
* **Decentralized Coordination:** Each follower drone estimates the leader's motion and maintains its own formation offset using exclusively local state estimates, eliminating single points of failure.
* **High-Fidelity SITL Simulation:** Built and rigorously validated using Gazebo Harmonic with synchronized rigid-body physics for Holybro X500 v2 airframes.

## Technical Stack

* **Operating System:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2 (Jazzy Jalisco)
* **Flight Stack:** PX4 Autopilot (v1.16)
* **Communication:** Micro XRCE-DDS Agent and Client
* **Simulator:** Gazebo Harmonic

## System Architecture

The control architecture unifies global reference frames through a local Cartesian NED frame based on a Flat-Earth Approximation. Follower velocity is dynamically scaled using a constant feedforward multiplier to reduce formation lag during dynamic maneuvers, while prioritizing safety-critical repulsion forces.

---

## Installation & Setup

### Prerequisites

Ensure you have the following installed and configured on your Ubuntu 24.04 system:

1. ROS 2 Jazzy
2. PX4 Autopilot (Specifically v1.16)
3. QGroundControl (Used strictly for flight state monitoring, not centralized command)
4. Micro XRCE-DDS bridge

### Cloning the Repository

```bash
git clone [https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git](https://github.com/swarmdrones-9/swarmdrones-with-interagent-collision-avoidance.git)
cd ws_offboard_control
