#!/usr/bin/env python3

"""
Path Controller for Multi-Vehicle Formation
Controls leader drone along predefined paths (square, circle, figure-8)
Logs all drone positions for post-processing analysis
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleGlobalPosition
import math
import csv
import argparse
from datetime import datetime
from typing import List, Tuple
import numpy as np
import threading
import subprocess
import os
import sys


class PathController(Node):
    """Node for controlling leader drone along predefined paths."""

    def __init__(self, path_type: str, num_drones: int = 5, 
                 square_size: float = 70, circle_radius: float = 5.0, 
                 figure8_size: float = 8.0, auto_visualize: bool = True) -> None:
        super().__init__('path_controller')

        # Configuration
        self.path_type = path_type.lower()
        self.num_drones = num_drones
        self.altitude = -5.0  # Constant altitude
        self.waypoint_tolerance = 0.1  # meters - larger tolerance for earlier waypoint switching (not used in path-following mode)
        self.path_following_lookahead = 5.0  # meters - look ahead distance along path for smooth following
        self.corner_radius = 0.0  # meters - how rounded the corners are (adjustable)
        self.corner_smoothing_enabled = True  # Enable/disable corner smoothing
        self.auto_visualize = auto_visualize  # Auto-run visualization when path completes
        
        # Path parameters
        self.square_size = square_size
        self.circle_radius = circle_radius
        self.figure8_size = figure8_size

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publisher for leader (px4_1)
        self.leader_namespace = "/px4_1"
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.leader_namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.leader_namespace}/fmu/in/trajectory_setpoint', qos_profile)

        # Spawn offset tracking - HARDCODED based on launch script
        # PX4_GZ_MODEL_POSE="x,y" format - testing different coordinate interpretations
        # Based on user feedback: followers 2&3 are ~7m off
        # Formation offsets (from C++): 
        #   Follower 1: (0, -3) - front-left
        #   Follower 2: (3, 0) - front-right  
        #   Follower 3: (0, 3) - back-left
        #   Follower 4: (-3, 0) - back-right
        #
        # Launch script spawns:
        #   Drone 1: PX4_GZ_MODEL_POSE="3,3"
        #   Drone 2: PX4_GZ_MODEL_POSE="-3,3"
        #   Drone 3: PX4_GZ_MODEL_POSE="3,-3"
        #   Drone 4: PX4_GZ_MODEL_POSE="-3,-3"
        #
        # If PX4_GZ_MODEL_POSE uses Gazebo ENU (x=East, y=North):
        #   "3,3" = 3m East, 3m North -> NED: [3.0, 3.0] (North, East)
        #   "-3,3" = -3m East, 3m North -> NED: [3.0, -3.0]
        #   "3,-3" = 3m East, -3m North -> NED: [-3.0, 3.0]
        #   "-3,-3" = -3m East, -3m North -> NED: [-3.0, -3.0]
        #
        # Formation offsets (from C++): 
        #   Drone 0 (Leader): (0, 0)
        #   Drone 1 (Follower 1): (0, -3) - front-left
        #   Drone 2 (Follower 2): (3, 0) - front-right (should be 3m North, 0m East)
        #   Drone 3 (Follower 3): (0, 3) - back-left (should be 0m North, 3m East)
        #   Drone 4 (Follower 4): (-3, 0) - back-right
        #
        # Launch script spawn positions (PX4_GZ_MODEL_POSE):
        #   Drone 1: "3,3" 
        #   Drone 2: "-3,3"  -> Should map to (3, 0) for Follower 2
        #   Drone 3: "3,-3"  -> Should map to (0, 3) for Follower 3
        #   Drone 4: "-3,-3"
        #
        # If PX4_GZ_MODEL_POSE uses Gazebo ENU (x=East, y=North):
        #   "-3,3" = -3m East, 3m North -> NED: [3.0, -3.0] but should be [3.0, 0.0]
        #   "3,-3" = 3m East, -3m North -> NED: [-3.0, 3.0] but should be [0.0, 3.0]
        #
        # Trying: Maybe PX4_GZ_MODEL_POSE is (North, East) in NED already, but values need adjustment
        # OR: The spawn positions are absolute, not relative offsets
        # SPAWN OFFSETS - Where drones actually spawn (from launch script PX4_GZ_MODEL_POSE)
        # Format: [North_offset, East_offset, Down_offset] in NED frame
        # These are the actual spawn positions relative to leader's spawn (0,0)
        #
        # Launch script PX4_GZ_MODEL_POSE values:
        #   Drone 1: "3,3"   -> Try: [3.0, 3.0] or [3.0, -3.0] or swap
        #   Drone 2: "-3,3"  -> Try: [3.0, -3.0] or [3.0, 3.0] or swap  
        #   Drone 3: "3,-3"  -> Try: [-3.0, 3.0] or [-3.0, -3.0] or swap
        #   Drone 4: "-3,-3" -> Try: [-3.0, -3.0] or [-3.0, 3.0] or swap
        #
        # ADJUST THESE VALUES if visualization shows wrong positions
        # If followers 2&3 are ~7m off, try different coordinate interpretations
        self.spawn_offsets = {
            0: [0.0, 0.0, 0.0],      # Leader - spawns at origin
            1: [3.0, 3.0, 0.0],      # Drone 1: PX4_GZ_MODEL_POSE="3,3" - ADJUST IF WRONG
            2: [3.0, -3.0, 0.0],    # Drone 2: PX4_GZ_MODEL_POSE="-3,3" - ADJUST IF WRONG (user says ~7m off)
            3: [-3.0, 3.0, 0.0],    # Drone 3: PX4_GZ_MODEL_POSE="3,-3" - ADJUST IF WRONG (user says ~7m off)
            4: [-3.0, -3.0, 0.0],   # Drone 4: PX4_GZ_MODEL_POSE="-3,-3" - ADJUST IF WRONG
        }
        self.spawn_offset_initialized = {}  # {drone_id: bool}
        self.spawn_offset_mutex = threading.Lock()
        
        # Mark all as initialized since we're using hardcoded values
        for i in range(num_drones):
            if i not in self.spawn_offsets:
                self.spawn_offsets[i] = [0.0, 0.0, 0.0]  # Default for any extra drones
            self.spawn_offset_initialized[i] = True
        
        self.get_logger().info("Using hardcoded spawn offsets:")
        for i in range(num_drones):
            offset = self.spawn_offsets[i]
            self.get_logger().info(f"  Drone {i}: [{offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f}] m")
        
        # GPS subscribers disabled - using hardcoded spawn offsets instead
        # Uncomment below if you want to use GPS-based detection again
        # self.gps_subscribers = []
        # for i in range(num_drones):
        #     instance_id = i + 1
        #     namespace = f"/px4_{instance_id}"
        #     topic = f'{namespace}/fmu/out/vehicle_global_position'
        #     
        #     gps_sub = self.create_subscription(
        #         VehicleGlobalPosition, topic,
        #         lambda msg, idx=i: self.gps_callback(idx, msg),
        #         qos_profile)
        #     self.gps_subscribers.append(gps_sub)
        
        # Subscribe to all drone positions for logging
        self.drone_positions = {}  # {drone_id: [x, y, z, timestamp]} in world frame
        self.drone_position_subscribers = []
        
        for i in range(num_drones):
            instance_id = i + 1
            namespace = f"/px4_{instance_id}"
            topic = f'{namespace}/fmu/out/vehicle_local_position'
            
            subscriber = self.create_subscription(
                VehicleLocalPosition, topic,
                lambda msg, idx=i: self.position_callback(idx, msg),
                qos_profile)
            self.drone_position_subscribers.append(subscriber)
            self.drone_positions[i] = None

        # Generate waypoints based on path type
        base_waypoints = self.generate_waypoints()
        # Smooth corners for square paths to reduce sharp turns
        if self.path_type == "square" and self.corner_smoothing_enabled:
            base_waypoints = self.smooth_corners(base_waypoints, corner_radius=self.corner_radius)
        # Add intermediate waypoints for smoother paths
        self.waypoints = self.interpolate_waypoints(base_waypoints, num_intermediate=0)
        self.current_waypoint_idx = 0
        self.path_complete = False
        self.should_exit = False
        self.exit_time = None
        
        # CSV logging setup - organize into plots/experiment_name/ folder
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_name = f"{self.path_type}_{timestamp}"
        
        # Create plots directory structure
        plots_dir = "plots"
        experiment_dir = os.path.join(plots_dir, experiment_name)
        os.makedirs(experiment_dir, exist_ok=True)
        
        # CSV file goes in the experiment folder
        self.csv_filename = os.path.join(experiment_dir, f"{experiment_name}.csv")
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.experiment_dir = experiment_dir  # Store for visualization
        
        # Write metadata as comment rows (using # prefix)
        # Visualization script can parse these
        self.csv_writer.writerow(['# Path metadata'])
        self.csv_writer.writerow(['# path_type', self.path_type])
        self.csv_writer.writerow(['# num_drones', self.num_drones])
        self.csv_writer.writerow(['# square_size', self.square_size])
        self.csv_writer.writerow(['# circle_radius', self.circle_radius])
        self.csv_writer.writerow(['# figure8_size', self.figure8_size])
        self.csv_writer.writerow(['# End metadata'])
        
        # Write CSV header
        self.csv_writer.writerow(['timestamp', 'drone_id', 'x', 'y', 'z', 'waypoint_idx'])
        
        # Write waypoints to CSV (drone_id = -1 for waypoints)
        # This allows visualization to plot the planned path
        for idx, waypoint in enumerate(self.waypoints):
            self.csv_writer.writerow([
                0.0,  # timestamp = 0 for waypoints
                -1,   # drone_id = -1 indicates waypoint
                waypoint[0],
                waypoint[1],
                waypoint[2],
                idx   # waypoint index
            ])
        
        self.get_logger().info(f"Path Controller initialized:")
        self.get_logger().info(f"  Path: {self.path_type}")
        self.get_logger().info(f"  Waypoints: {len(self.waypoints)}")
        self.get_logger().info(f"  Experiment folder: {self.experiment_dir}")
        self.get_logger().info(f"  CSV file: {self.csv_filename}")
        self.get_logger().info(f"  Auto-visualize: {self.auto_visualize}")

        # Wait a bit for C++ to handle takeoff
        self.start_time = self.get_clock().now()
        self.initialization_delay = 0  # seconds - wait for C++ takeoff
        
        # Create high-frequency timer (50 Hz = 0.02s)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def generate_waypoints(self) -> List[Tuple[float, float, float]]:
        """Generate waypoints for the selected path type."""
        waypoints = []
        
        if self.path_type == "square":
            # Square path: start at origin, go around square
            half_size = self.square_size / 2.0
            waypoints = [
                (half_size, -half_size, self.altitude),   # Top-right
                (half_size, half_size, self.altitude),   # Bottom-right
                (-half_size, half_size, self.altitude),  # Bottom-left
                (-half_size, -half_size, self.altitude), # Top-left
                (half_size, -half_size, self.altitude),  # Back to start
            ]
            
        elif self.path_type == "circle":
            # Circle path: generate points around circle
            num_points = 32  # Smooth circle
            for i in range(num_points + 1):  # +1 to close the circle
                angle = 2.0 * math.pi * i / num_points
                x = self.circle_radius * math.cos(angle)
                y = self.circle_radius * math.sin(angle)
                waypoints.append((x, y, self.altitude))
                
        elif self.path_type == "figure8":
            # Figure-8 path: two circles connected
            num_points = 32
            for i in range(num_points + 1):
                angle = 2.0 * math.pi * i / num_points
                # Figure-8 parametric equations
                x = self.figure8_size * math.sin(angle)
                y = self.figure8_size * math.sin(angle) * math.cos(angle)
                waypoints.append((x, y, self.altitude))
                
        else:
            self.get_logger().error(f"Unknown path type: {self.path_type}")
            waypoints = [(0.0, 0.0, self.altitude)]
            
        return waypoints

    def smooth_corners(self, waypoints: List[Tuple[float, float, float]], 
                      corner_radius: float = None) -> List[Tuple[float, float, float]]:
        """Add rounded corners to square paths to reduce sharp turns.
        
        Args:
            waypoints: List of waypoint tuples (x, y, z)
            corner_radius: How rounded the corners are in meters (defaults to self.corner_radius)
        
        Returns:
            List of waypoints with rounded corners inserted
        """
        if corner_radius is None:
            corner_radius = self.corner_radius
            
        if self.path_type != "square" or len(waypoints) < 4:
            return waypoints
        
        smoothed = []
        
        for i in range(len(waypoints)):
            current = waypoints[i]
            next_wp = waypoints[(i + 1) % len(waypoints)]
            prev_wp = waypoints[(i - 1) % len(waypoints)]
            
            # Calculate direction vectors
            dir_to_current = (
                current[0] - prev_wp[0],
                current[1] - prev_wp[1],
                current[2] - prev_wp[2]
            )
            dir_from_current = (
                next_wp[0] - current[0],
                next_wp[1] - current[1],
                next_wp[2] - current[2]
            )
            
            # Normalize direction vectors
            dist_to = math.sqrt(dir_to_current[0]**2 + dir_to_current[1]**2 + dir_to_current[2]**2)
            dist_from = math.sqrt(dir_from_current[0]**2 + dir_from_current[1]**2 + dir_from_current[2]**2)
            
            if dist_to > 0.1 and dist_from > 0.1:
                # Create entry and exit points for rounded corner
                # Use 30% of segment length, but limit by corner_radius
                entry_dist = min(corner_radius, dist_to * 0.3)
                exit_dist = min(corner_radius, dist_from * 0.3)
                
                entry_point = (
                    current[0] - dir_to_current[0] / dist_to * entry_dist,
                    current[1] - dir_to_current[1] / dist_to * entry_dist,
                    current[2] - dir_to_current[2] / dist_to * entry_dist
                )
                
                exit_point = (
                    current[0] + dir_from_current[0] / dist_from * exit_dist,
                    current[1] + dir_from_current[1] / dist_from * exit_dist,
                    current[2] + dir_from_current[2] / dist_from * exit_dist
                )
                
                # Only add entry point if it's different from previous exit point
                if len(smoothed) == 0 or smoothed[-1] != entry_point:
                    smoothed.append(entry_point)
                smoothed.append(exit_point)
            else:
                smoothed.append(current)
        
        return smoothed

    def interpolate_waypoints(self, waypoints: List[Tuple[float, float, float]], 
                             num_intermediate: int = 2) -> List[Tuple[float, float, float]]:
        """Add intermediate waypoints between main waypoints for smoother paths."""
        if len(waypoints) < 2:
            return waypoints
        
        interpolated = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            interpolated.append(start)  # Add the waypoint itself
            
            # Add intermediate points
            for j in range(1, num_intermediate + 1):
                t = j / (num_intermediate + 1)
                x = start[0] + (end[0] - start[0]) * t
                y = start[1] + (end[1] - start[1]) * t
                z = start[2] + (end[2] - start[2]) * t
                interpolated.append((x, y, z))
        
        interpolated.append(waypoints[-1])  # Add final waypoint
        return interpolated

    def gps_callback(self, drone_id: int, msg: VehicleGlobalPosition):
        """GPS callback to auto-detect spawn offsets (same logic as C++ code)."""
        with self.spawn_offset_mutex:
            # Only process if not yet initialized and GPS data is valid
            if self.spawn_offset_initialized[drone_id]:
                return  # Already initialized
            
            # Check if GPS has valid fix (lat/lon not zero)
            if abs(msg.lat) < 1e-6 and abs(msg.lon) < 1e-6:
                return  # Invalid GPS data
            
            if drone_id == 0:
                # Leader (drone 0) sets the reference point
                self.reference_lat_lon[0] = msg.lat
                self.reference_lat_lon[1] = msg.lon
                self.reference_set = True
                self.spawn_offsets[0] = [0.0, 0.0, 0.0]
                self.spawn_offset_initialized[0] = True
                
                self.get_logger().info(
                    f"Leader (drone 0) GPS reference: lat={msg.lat:.7f}, lon={msg.lon:.7f}")
            elif self.reference_set:
                # Followers calculate their offset from the leader's reference point
                # Convert lat/lon difference to local NED coordinates (meters)
                lat_diff = msg.lat - self.reference_lat_lon[0]
                lon_diff = msg.lon - self.reference_lat_lon[1]
                
                x_offset = float(lat_diff * 111320.0)  # North (meters)
                y_offset = float(lon_diff * 111320.0 * 
                                math.cos(self.reference_lat_lon[0] * math.pi / 180.0))  # East (meters)
                
                self.spawn_offsets[drone_id] = [x_offset, y_offset, 0.0]
                self.spawn_offset_initialized[drone_id] = True
                
                self.get_logger().info(
                    f"Drone {drone_id} spawn offset detected: [{x_offset:.2f}, {y_offset:.2f}, 0.0] m from leader")
                self.get_logger().info(
                    f"  GPS: lat={msg.lat:.7f}, lon={msg.lon:.7f} | "
                    f"Reference: lat={self.reference_lat_lon[0]:.7f}, lon={self.reference_lat_lon[1]:.7f}")

    def position_callback(self, drone_id: int, msg: VehicleLocalPosition):
        """Callback for drone position updates - converts to world frame before storing."""
        if msg.xy_valid and msg.z_valid:
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # Convert to world frame by adding hardcoded spawn offset
            with self.spawn_offset_mutex:
                offset = self.spawn_offsets.get(drone_id, [0.0, 0.0, 0.0])
            
            world_x = msg.x + offset[0]
            world_y = msg.y + offset[1]
            world_z = msg.z + offset[2]
            
            self.drone_positions[drone_id] = {
                'x': world_x,
                'y': world_y,
                'z': world_z,
                'timestamp': timestamp
            }

    def log_positions(self, waypoint_idx: int):
        """Log all drone positions to CSV."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        for drone_id, pos_data in self.drone_positions.items():
            if pos_data is not None:
                self.csv_writer.writerow([
                    timestamp,
                    drone_id,
                    pos_data['x'],
                    pos_data['y'],
                    pos_data['z'],
                    waypoint_idx
                ])

    def distance_to_waypoint(self, x: float, y: float, z: float, 
                            waypoint: Tuple[float, float, float]) -> float:
        """Calculate distance to waypoint."""
        dx = x - waypoint[0]
        dy = y - waypoint[1]
        dz = z - waypoint[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_current_leader_position(self) -> Tuple[float, float, float]:
        """Get current leader position (drone 0)."""
        if self.drone_positions[0] is not None:
            return (self.drone_positions[0]['x'],
                   self.drone_positions[0]['y'],
                   self.drone_positions[0]['z'])
        return (0.0, 0.0, 0.0)

    def timer_callback(self) -> None:
        """High-frequency timer callback for path following."""
        # Wait for initialization delay
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.initialization_delay:
            return

        # Check if path is complete
        if self.path_complete:
            # Keep logging for a bit, then signal exit
            self.log_positions(self.current_waypoint_idx)
            if self.exit_time is None:
                self.exit_time = elapsed
            if elapsed > self.exit_time + 2.0:  # Log for 2 more seconds
                self.get_logger().info("Path complete. Closing CSV...")
                self.csv_file.close()
                
                # Auto-visualize if enabled
                if self.auto_visualize:
                    self.visualize_path()
                
                self.should_exit = True
                return
            return

        # Publish offboard control mode heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False  # Disabled - using position-only control
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_msg)

        # Get current leader position
        current_pos = self.get_current_leader_position()
        
        # Fast discrete waypoint switching - switch waypoints early for smooth movement
        if len(self.waypoints) == 0:
            return
        
        # Find which waypoint we're targeting
        current_wp_idx = self.current_waypoint_idx % len(self.waypoints)
        target_waypoint = self.waypoints[current_wp_idx]
        
        # Calculate distance to current target waypoint
        dx = target_waypoint[0] - current_pos[0]
        dy = target_waypoint[1] - current_pos[1]
        dz = target_waypoint[2] - current_pos[2]
        dist_to_wp = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Switch waypoint early (large tolerance) for faster, smoother movement
        # Check if we're approaching the waypoint from the front or already passed it
        # Calculate direction vector from previous waypoint
        if current_wp_idx > 0:
            prev_wp = self.waypoints[current_wp_idx - 1]
        else:
            prev_wp = self.waypoints[-1]  # Wrap around
        
        wp_direction = (target_waypoint[0] - prev_wp[0],
                       target_waypoint[1] - prev_wp[1],
                       target_waypoint[2] - prev_wp[2])
        wp_dir_len = math.sqrt(wp_direction[0]**2 + wp_direction[1]**2 + wp_direction[2]**2)
        
        # Check if we've passed the waypoint or are close enough
        passed_waypoint = False
        if wp_dir_len > 0.01:
            # Vector from waypoint to current position
            to_current = (current_pos[0] - target_waypoint[0],
                         current_pos[1] - target_waypoint[1],
                         current_pos[2] - target_waypoint[2])
            
            # Dot product: positive means we're past the waypoint in the direction of travel
            dot_product = (to_current[0] * wp_direction[0] + 
                          to_current[1] * wp_direction[1] + 
                          to_current[2] * wp_direction[2]) / wp_dir_len
            
            # If we're ahead of waypoint in direction of travel, we've passed it
            passed_waypoint = dot_product > 0
        else:
            # If waypoints are too close, just use distance
            passed_waypoint = dist_to_wp < 1.0
        
        # Switch waypoint if: close enough OR already passed it
        switch_waypoint = dist_to_wp < 5.0 or passed_waypoint  # Large tolerance for early switching
        
        if switch_waypoint:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                # Completed one lap
                self.path_complete = True
                self.get_logger().info("Path completed!")
                return
            else:
                # Update to next waypoint
                target_waypoint = self.waypoints[self.current_waypoint_idx]
                dx = target_waypoint[0] - current_pos[0]
                dy = target_waypoint[1] - current_pos[1]
                dz = target_waypoint[2] - current_pos[2]
                dist_to_wp = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Calculate look-ahead point for smooth movement
        look_ahead_dist = 2.0  # meters ahead
        if dist_to_wp > 0.01:
            dir_x = dx / dist_to_wp
            dir_y = dy / dist_to_wp
            dir_z = dz / dist_to_wp
            
            look_ahead_x = target_waypoint[0] + dir_x * look_ahead_dist
            look_ahead_y = target_waypoint[1] + dir_y * look_ahead_dist
            look_ahead_z = target_waypoint[2] + dir_z * look_ahead_dist
        else:
            look_ahead_x = target_waypoint[0]
            look_ahead_y = target_waypoint[1]
            look_ahead_z = target_waypoint[2]
        
        # Publish trajectory setpoint
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.position = [look_ahead_x, look_ahead_y, look_ahead_z]
        setpoint_msg.velocity = [float('nan'), float('nan'), float('nan')]  # Disable velocity control
        setpoint_msg.yaw = 0.0
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(setpoint_msg)

        # Log all positions
        self.log_positions(self.current_waypoint_idx)

    def visualize_path(self):
        """Automatically run visualization script on the CSV file."""
        try:
            # Get the directory of this script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            visualize_script = os.path.join(script_dir, 'visualize_path.py')
            csv_path = os.path.abspath(self.csv_filename)
            
            self.get_logger().info(f"Running visualization for {self.csv_filename}...")
            
            # Run visualization script - output plots to the same experiment folder as CSV
            result = subprocess.run(
                [sys.executable, visualize_script, csv_path, 
                 '--num_drones', str(self.num_drones),
                 '--output_dir', self.experiment_dir,
                 '--no-show'],  # Don't show plots, just save them
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                self.get_logger().info("Visualization completed successfully!")
                self.get_logger().info(f"Plots saved in: {self.experiment_dir}")
                if result.stdout:
                    # Print visualization output
                    print(result.stdout)
            else:
                self.get_logger().warn(f"Visualization had issues:")
                if result.stderr:
                    self.get_logger().warn(result.stderr)
                if result.stdout:
                    self.get_logger().warn(result.stdout)
                
        except Exception as e:
            self.get_logger().error(f"Failed to run visualization: {e}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Path Controller for Multi-Vehicle Formation')
    parser.add_argument('--path', type=str, default='square',
                       choices=['square', 'circle', 'figure8'],
                       help='Path type: square, circle, or figure8')
    parser.add_argument('--num_drones', type=int, default=5,
                       help='Number of drones')
    parser.add_argument('--square_size', type=float, default=70.0,
                       help='Square path size in meters')
    parser.add_argument('--circle_radius', type=float, default=5.0,
                       help='Circle path radius in meters')
    parser.add_argument('--figure8_size', type=float, default=8.0,
                       help='Figure-8 path size in meters')
    parser.add_argument('--no-visualize', action='store_true',
                       help='Disable automatic visualization after path completion')
    
    args = parser.parse_args()
    
    print('Starting Path Controller...')
    print(f'Path: {args.path}')
    
    rclpy.init(args=None)
    path_controller = PathController(
        path_type=args.path,
        num_drones=args.num_drones,
        square_size=args.square_size,
        circle_radius=args.circle_radius,
        figure8_size=args.figure8_size,
        auto_visualize=not args.no_visualize  # Enable by default, disable with --no-visualize
    )
    
    try:
        # Spin with timeout to check for exit flag
        while rclpy.ok() and not path_controller.should_exit:
            rclpy.spin_once(path_controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if not path_controller.csv_file.closed:
            path_controller.csv_file.close()
        path_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

