/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Multi-vehicle offboard control example
 * @file multi_vehicle_offboard_control.cpp
 * @addtogroup examples
 * @brief Controls multiple PX4 vehicles simultaneously from a single node
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <stdint.h>
#include <vector>
#include <string>

#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class MultiVehicleOffboardControl : public rclcpp::Node
{
public:
	MultiVehicleOffboardControl(uint8_t num_drones) 
		: Node("multi_vehicle_offboard_control"),
		  num_drones_(num_drones)
	{
		// ============================================================
		// COLLISION AVOIDANCE & PREDICTIVE CONTROL PARAMETERS
		// ============================================================
		// Adjust all parameters here in one place
		collision_avoidance_enabled_ = true;          // Enable/disable collision avoidance
		use_predictive_control_ = false;                // Enable/disable predictive following
		
		// Collision avoidance parameters
		min_separation_distance_ = 1.5f;              // meters - minimum safe distance
		prediction_horizon_ = 2.0f;                   // seconds - how far ahead to predict collisions
		max_repulsion_force_ = 10.0f;                   // meters - max deviation from formation
		closure_rate_threshold_ = 0.3f;               // m/s - ignore slower approaches
		altitude_separation_distance_ = 1.5f;        // meters - vertical separation backup
		
		// Predictive following parameters
		prediction_time_ = 0.3f;                       // seconds - how far ahead to predict leader position
		
		// Follower velocity control parameters
		follower_velocity_multiplier_ = 1.10f;          // Multiplier for follower speed (1.0 = same as leader, 1.1 = 10% faster, 0.9 = 10% slower)
		enable_follower_velocity_control_ = true;      // Enable/disable velocity control for followers
		// ============================================================
		
		// Initialize vectors to track state for each drone
		setpoint_counters_.resize(num_drones_, 0);
		armed_.resize(num_drones_, false);
		in_offboard_mode_.resize(num_drones_, false);
		
		// Initialize GPS-based spawn detection
		spawn_offsets_.resize(num_drones_, {0.0f, 0.0f, 0.0f});
		spawn_offset_initialized_.resize(num_drones_, false);
		reference_set_ = false;
		
		// Diamond formation offsets relative to leader
		formation_offsets_ = {
			{0.0f, 0.0f, 0.0f},                    // Leader at center
			{-3.0f, -3.0f, 0.0f},                   // Follower 1: front-left
			{3.0f, 3.0f, 0.0f},                  // Follower 2: front-right
			{-3.0f, 3.0f, 0.0f},                  // Follower 3: back-left
			{3.0f, -3.0f, 0.0f}                  // Follower 4: back-right
		};
		
		// Initialize leader position tracking
		leader_world_pos_ = {0.0f, 0.0f, -5.0f};
		leader_velocity_ = {0.0f, 0.0f, 0.0f};
		leader_position_received_ = false;
		leader_yaw_ = 0.0f;
		leader_attitude_received_ = false;
		leader_position_time_ = rclcpp::Time(0);
		
		// Initialize collision avoidance state tracking
		drone_positions_.resize(num_drones_, {0.0f, 0.0f, 0.0f});
		drone_velocities_.resize(num_drones_, {0.0f, 0.0f, 0.0f});
		drone_position_times_.resize(num_drones_);
		drone_position_received_.resize(num_drones_, false);
		
		// Initialize start time for delay before formation
		start_time_ = this->get_clock()->now();
		
		// Create publishers for each drone
		// Note: PX4 instances start from 1, not 0
		// drone_id 0 → instance 1, drone_id 1 → instance 2, etc.
		for (uint8_t i = 0; i < num_drones_; i++) {
			// Map drone_id to PX4 instance_id (instances start from 1)
			uint8_t instance_id = i + 1;
			
			// Build namespace for this drone
			// All instances have namespace /px4_{instance_id}
			std::string ns = "/px4_" + std::to_string(instance_id);
			
			// Ensure namespace ends with '/' for proper topic construction
			std::string topic_ns = ns;
			if (!topic_ns.empty() && topic_ns.back() != '/') {
				topic_ns += "/";
			}
			
			// Create publishers for this drone
			offboard_control_mode_publishers_.push_back(
				this->create_publisher<OffboardControlMode>(
					topic_ns + "fmu/in/offboard_control_mode", 10));
			
			trajectory_setpoint_publishers_.push_back(
				this->create_publisher<TrajectorySetpoint>(
					topic_ns + "fmu/in/trajectory_setpoint", 10));
			
			vehicle_command_publishers_.push_back(
				this->create_publisher<VehicleCommand>(
					topic_ns + "fmu/in/vehicle_command", 10));
			
			// According to PX4 docs: target_system = instance_id + 1
			uint8_t target_system = instance_id + 1;
			RCLCPP_INFO(this->get_logger(), "Initialized control for drone %d (instance %d, namespace: %s, target_system: %d)", 
				i, instance_id, ns.c_str(), target_system);
		}
		
		RCLCPP_INFO(this->get_logger(), "Multi-vehicle offboard control initialized for %d drones", num_drones_);
		
		// Subscribe to leader's position (instance 1)
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		leader_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"/px4_1/fmu/out/vehicle_local_position", qos,
			std::bind(&MultiVehicleOffboardControl::leader_position_callback, this, std::placeholders::_1));
		
		RCLCPP_INFO(this->get_logger(), "Subscribed to leader position: /px4_1/fmu/out/vehicle_local_position");
		
		// Subscribe to leader's attitude (instance 1) to get yaw
		leader_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"/px4_1/fmu/out/vehicle_attitude", qos,
			std::bind(&MultiVehicleOffboardControl::leader_attitude_callback, this, std::placeholders::_1));
		
		RCLCPP_INFO(this->get_logger(), "Subscribed to leader attitude: /px4_1/fmu/out/vehicle_attitude");
		
		// Subscribe to GPS for all drones to auto-detect spawn offsets
		RCLCPP_INFO(this->get_logger(), "Setting up GPS-based spawn offset detection...");
		for (uint8_t i = 0; i < num_drones_; i++) {
			uint8_t instance_id = i + 1;
			std::string ns = "/px4_" + std::to_string(instance_id);
			if (!ns.empty() && ns.back() != '/') {
				ns += "/";
			}
			
			// Subscribe to GPS position to detect spawn offset
			auto gps_sub = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
				ns + "fmu/out/vehicle_global_position", qos,
				[this, i](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
					this->gps_callback(i, msg);
				});
			gps_subscribers_.push_back(gps_sub);
			
			RCLCPP_INFO(this->get_logger(), "Subscribed to GPS for drone %d: %s", 
				i, (ns + "fmu/out/vehicle_global_position").c_str());
		}
		
		// Subscribe to all drones' positions for collision avoidance
		RCLCPP_INFO(this->get_logger(), "Setting up position tracking for collision avoidance...");
		for (uint8_t i = 0; i < num_drones_; i++) {
			uint8_t instance_id = i + 1;
			std::string ns = "/px4_" + std::to_string(instance_id);
			if (!ns.empty() && ns.back() != '/') {
				ns += "/";
			}
			
			// Subscribe to position for collision avoidance
			auto pos_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
				ns + "fmu/out/vehicle_local_position", qos,
				[this, i](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
					this->drone_position_callback(i, msg);
				});
			drone_position_subscribers_.push_back(pos_sub);
		}
		
		RCLCPP_INFO(this->get_logger(), "Collision avoidance: %s", 
			collision_avoidance_enabled_ ? "ENABLED" : "DISABLED");
		RCLCPP_INFO(this->get_logger(), "Predictive control: %s", 
			use_predictive_control_ ? "ENABLED" : "DISABLED");
		
		// Create timer that handles all drones
		timer_ = this->create_wall_timer(20ms, std::bind(&MultiVehicleOffboardControl::timer_callback, this));
	}

private:
	uint8_t num_drones_;  //!< Number of drones to control
	
	// ============================================================
	// COLLISION AVOIDANCE & PREDICTIVE CONTROL PARAMETERS
	// ============================================================
	bool collision_avoidance_enabled_;
	bool use_predictive_control_;
	float min_separation_distance_;
	float prediction_horizon_;
	float max_repulsion_force_;
	float closure_rate_threshold_;
	float altitude_separation_distance_;
	float prediction_time_;
	
	// Follower velocity control
	float follower_velocity_multiplier_;
	bool enable_follower_velocity_control_;
	// ============================================================
	
	// Publishers for each drone
	std::vector<rclcpp::Publisher<OffboardControlMode>::SharedPtr> offboard_control_mode_publishers_;
	std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> trajectory_setpoint_publishers_;
	std::vector<rclcpp::Publisher<VehicleCommand>::SharedPtr> vehicle_command_publishers_;
	
	// State tracking for each drone
	std::vector<uint64_t> setpoint_counters_;  //!< Counter for setpoints sent to each drone
	std::vector<bool> armed_;                  //!< Armed status for each drone
	std::vector<bool> in_offboard_mode_;       //!< Offboard mode status for each drone
	
	// Formation configuration
	std::vector<std::array<float, 3>> spawn_offsets_;      //!< Auto-detected spawn positions
	std::vector<bool> spawn_offset_initialized_;            //!< Whether spawn offset has been detected
	std::vector<std::array<float, 3>> formation_offsets_;  //!< Formation offsets relative to leader
	rclcpp::Time start_time_;                               //!< Start time for delay before formation
	
	// GPS-based spawn detection
	std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr> gps_subscribers_;
	std::array<double, 2> reference_lat_lon_;  //!< Reference GPS point (leader's spawn)
	bool reference_set_;                        //!< Whether reference GPS point has been set
	std::mutex spawn_offset_mutex_;             //!< Mutex for thread-safe spawn offset access
	
	// Leader position tracking
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr leader_position_subscriber_;
	std::array<float, 3> leader_world_pos_;  //!< Leader position in world frame
	std::array<float, 3> leader_velocity_;  //!< Leader velocity in world frame
	bool leader_position_received_;          //!< Flag if leader position has been received
	rclcpp::Time leader_position_time_;       //!< Time of last leader position update
	std::mutex leader_pos_mutex_;            //!< Mutex for thread-safe access to leader position
	
	// Leader attitude tracking
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr leader_attitude_subscriber_;
	float leader_yaw_;                       //!< Leader yaw in radians
	bool leader_attitude_received_;          //!< Flag if leader attitude has been received
	std::mutex leader_yaw_mutex_;            //!< Mutex for thread-safe access to leader yaw
	
	// Position and velocity tracking for all drones (for collision avoidance)
	std::vector<std::array<float, 3>> drone_positions_;  //!< World frame positions
	std::vector<std::array<float, 3>> drone_velocities_;  //!< World frame velocities
	std::vector<rclcpp::Time> drone_position_times_;  //!< For velocity calculation
	std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr> drone_position_subscribers_;
	std::vector<bool> drone_position_received_;  //!< Track if position received
	std::mutex drone_state_mutex_;  //!< Mutex for thread-safe access to drone positions/velocities
	
	rclcpp::TimerBase::SharedPtr timer_;       //!< Timer for periodic control
	
	/**
	 * @brief Callback for leader position updates
	 * @param msg VehicleLocalPosition message from leader
	 */
	void leader_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
		if (msg->xy_valid && msg->z_valid) {
			std::lock_guard<std::mutex> lock(leader_pos_mutex_);
			std::lock_guard<std::mutex> lock2(spawn_offset_mutex_);
			
			rclcpp::Time current_time = this->get_clock()->now();
			
			// Convert from local NED frame to world frame by adding spawn offset
			std::array<float, 3> new_position = {
				msg->x + spawn_offsets_[0][0],
				msg->y + spawn_offsets_[0][1],
				msg->z + spawn_offsets_[0][2]
			};
			
			// Calculate velocity for predictive control
			if (leader_position_received_ && leader_position_time_.nanoseconds() > 0) {
				double dt = (current_time - leader_position_time_).seconds();
				if (dt > 0.0 && dt < 1.0) {  // Sanity check
					leader_velocity_[0] = (new_position[0] - leader_world_pos_[0]) / dt;
					leader_velocity_[1] = (new_position[1] - leader_world_pos_[1]) / dt;
					leader_velocity_[2] = (new_position[2] - leader_world_pos_[2]) / dt;
				}
			}
			
			leader_world_pos_ = new_position;
			leader_position_time_ = current_time;
			leader_position_received_ = true;
		}
	}
	
	/**
	 * @brief Callback for leader's attitude (to get yaw)
	 */
	void leader_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
	{
		// Extract yaw from quaternion (q[0]=w, q[1]=x, q[2]=y, q[3]=z)
		float q0 = msg->q[0];  // w
		float q1 = msg->q[1];  // x
		float q2 = msg->q[2];  // y
		float q3 = msg->q[3];  // z
		
		// Convert quaternion to yaw (NED frame)
		float yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
		
		std::lock_guard<std::mutex> lock(leader_yaw_mutex_);
		leader_yaw_ = yaw;
		leader_attitude_received_ = true;
	}
	
	/**
	 * @brief GPS callback to auto-detect spawn offsets
	 * @param drone_id The drone index (0 to num_drones-1)
	 * @param msg GPS position message
	 */
	void gps_callback(uint8_t drone_id, const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(spawn_offset_mutex_);
		
		// Only process if not yet initialized and GPS data is valid
		if (spawn_offset_initialized_[drone_id]) {
			return;  // Already initialized
		}
		
		// Check if GPS has valid fix (lat/lon not zero)
		if (std::abs(msg->lat) < 1e-6 && std::abs(msg->lon) < 1e-6) {
			return;  // Invalid GPS data
		}
		
		if (drone_id == 0) {
			// Leader (drone 0) sets the reference point
			reference_lat_lon_[0] = msg->lat;
			reference_lat_lon_[1] = msg->lon;
			reference_set_ = true;
			spawn_offsets_[0] = {0.0f, 0.0f, 0.0f};
			spawn_offset_initialized_[0] = true;
			
			RCLCPP_INFO(this->get_logger(), 
				"Leader (drone 0) GPS reference: lat=%.7f, lon=%.7f", 
				msg->lat, msg->lon);
		} else if (reference_set_) {
			// Followers calculate their offset from the leader's reference point
			// Convert lat/lon difference to local NED coordinates (meters)
			double lat_diff = msg->lat - reference_lat_lon_[0];
			double lon_diff = msg->lon - reference_lat_lon_[1];
			
			float x_offset = static_cast<float>(lat_diff * 111320.0);  // North (meters)
			float y_offset = static_cast<float>(lon_diff * 111320.0 * 
				std::cos(reference_lat_lon_[0] * M_PI / 180.0));  // East (meters)
			
			spawn_offsets_[drone_id] = {x_offset, y_offset, 0.0f};
			spawn_offset_initialized_[drone_id] = true;
			
			RCLCPP_INFO(this->get_logger(), 
				"Drone %d spawn offset: [%.2f, %.2f, 0.0] m from leader", 
				drone_id, x_offset, y_offset);
		}
	}
	
	/**
	 * @brief Callback for drone position updates (for collision avoidance)
	 * @param drone_id The drone index (0 to num_drones-1)
	 * @param msg VehicleLocalPosition message
	 */
	void drone_position_callback(uint8_t drone_id, const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
		if (!msg->xy_valid || !msg->z_valid) {
			return;
		}
		
		std::lock_guard<std::mutex> lock(drone_state_mutex_);
		std::lock_guard<std::mutex> lock2(spawn_offset_mutex_);
		
		rclcpp::Time current_time = this->get_clock()->now();
		
		// Convert to world frame
		std::array<float, 3> new_position = {
			msg->x + spawn_offsets_[drone_id][0],
			msg->y + spawn_offsets_[drone_id][1],
			msg->z + spawn_offsets_[drone_id][2]
		};
		
		// Calculate velocity
		if (drone_position_received_[drone_id] && drone_position_times_[drone_id].nanoseconds() > 0) {
			double dt = (current_time - drone_position_times_[drone_id]).seconds();
			if (dt > 0.0 && dt < 1.0) {  // Sanity check
				drone_velocities_[drone_id][0] = (new_position[0] - drone_positions_[drone_id][0]) / dt;
				drone_velocities_[drone_id][1] = (new_position[1] - drone_positions_[drone_id][1]) / dt;
				drone_velocities_[drone_id][2] = (new_position[2] - drone_positions_[drone_id][2]) / dt;
			}
		}
		
		drone_positions_[drone_id] = new_position;
		drone_position_times_[drone_id] = current_time;
		drone_position_received_[drone_id] = true;
	}
	
	/**
	 * @brief Apply velocity-based predictive collision avoidance
	 * @param drone_id The drone index (0 to num_drones-1)
	 * @param target Base target position in world frame
	 * @return Adjusted target position with collision avoidance
	 */
	std::array<float, 3> apply_collision_avoidance(uint8_t drone_id, const std::array<float, 3>& target)
	{
		if (!collision_avoidance_enabled_) {
			return target;
		}
		
		std::lock_guard<std::mutex> lock(drone_state_mutex_);
		
		std::array<float, 3> current_pos = drone_positions_[drone_id];
		std::array<float, 3> current_vel = drone_velocities_[drone_id];
		std::array<float, 3> adjusted_target = target;
		std::array<float, 3> total_repulsion = {0.0f, 0.0f, 0.0f};
		
		// Check all other drones
		for (uint8_t other_id = 0; other_id < num_drones_; other_id++) {
			if (other_id == drone_id || !drone_position_received_[other_id]) {
				continue;
			}
			
			std::array<float, 3> other_pos = drone_positions_[other_id];
			std::array<float, 3> other_vel = drone_velocities_[other_id];
			
			// Calculate relative position and velocity
			std::array<float, 3> rel_pos = {
				other_pos[0] - current_pos[0],
				other_pos[1] - current_pos[1],
				other_pos[2] - current_pos[2]
			};
			
			std::array<float, 3> rel_vel = {
				other_vel[0] - current_vel[0],
				other_vel[1] - current_vel[1],
				other_vel[2] - current_vel[2]
			};
			
			// Calculate current distance
			float current_distance = std::sqrt(
				rel_pos[0] * rel_pos[0] + 
				rel_pos[1] * rel_pos[1] + 
				rel_pos[2] * rel_pos[2]
			);
			
			// Skip if too far away
			if (current_distance > min_separation_distance_ * 3.0f) {
				continue;
			}
			
			// Calculate closure rate
			float closure_rate = 0.0f;
			if (current_distance > 0.01f) {
				// Dot product of relative velocity and relative position unit vector
				std::array<float, 3> rel_pos_unit = {
					rel_pos[0] / current_distance,
					rel_pos[1] / current_distance,
					rel_pos[2] / current_distance
				};
				closure_rate = rel_vel[0] * rel_pos_unit[0] + 
				               rel_vel[1] * rel_pos_unit[1] + 
				               rel_vel[2] * rel_pos_unit[2];
			}
			
			// Predict future positions
			std::array<float, 3> predicted_pos = {
				current_pos[0] + current_vel[0] * prediction_horizon_,
				current_pos[1] + current_vel[1] * prediction_horizon_,
				current_pos[2] + current_vel[2] * prediction_horizon_
			};
			
			std::array<float, 3> predicted_other_pos = {
				other_pos[0] + other_vel[0] * prediction_horizon_,
				other_pos[1] + other_vel[1] * prediction_horizon_,
				other_pos[2] + other_vel[2] * prediction_horizon_
			};
			
			// Calculate predicted distance
			std::array<float, 3> predicted_rel_pos = {
				predicted_other_pos[0] - predicted_pos[0],
				predicted_other_pos[1] - predicted_pos[1],
				predicted_other_pos[2] - predicted_pos[2]
			};
			float predicted_distance = std::sqrt(
				predicted_rel_pos[0] * predicted_rel_pos[0] + 
				predicted_rel_pos[1] * predicted_rel_pos[1] + 
				predicted_rel_pos[2] * predicted_rel_pos[2]
			);
			
			// Determine if this is a threat
			bool is_closing = closure_rate < -closure_rate_threshold_;
			bool will_be_close = predicted_distance < min_separation_distance_;
			bool already_close = current_distance < min_separation_distance_;
			
			if ((is_closing && will_be_close) || already_close) {
				// Calculate urgency
				float time_to_collision = prediction_horizon_;
				if (is_closing && current_distance > 0.01f) {
					time_to_collision = (current_distance - min_separation_distance_) / std::abs(closure_rate);
					time_to_collision = std::max(0.1f, std::min(time_to_collision, prediction_horizon_));
				}
				
				float distance_urgency = std::max(0.0f, 
					(min_separation_distance_ - current_distance) / min_separation_distance_);
				float time_urgency = std::max(0.0f, 
					(prediction_horizon_ - time_to_collision) / prediction_horizon_);
				float velocity_urgency = std::min(1.0f, std::abs(closure_rate) / 2.0f);
				
				float urgency = distance_urgency * 0.4f + time_urgency * 0.3f + velocity_urgency * 0.3f;
				urgency = std::max(0.1f, std::min(1.0f, urgency));
				
				// Calculate repulsion direction (away from other drone)
				if (current_distance > 0.01f) {
					std::array<float, 3> repulsion_dir = {
						current_pos[0] - other_pos[0],
						current_pos[1] - other_pos[1],
						current_pos[2] - other_pos[2]
					};
					
					float dir_magnitude = std::sqrt(
						repulsion_dir[0] * repulsion_dir[0] + 
						repulsion_dir[1] * repulsion_dir[1] + 
						repulsion_dir[2] * repulsion_dir[2]
					);
					
					if (dir_magnitude > 0.0f) {
						repulsion_dir[0] /= dir_magnitude;
						repulsion_dir[1] /= dir_magnitude;
						repulsion_dir[2] /= dir_magnitude;
						
						// Repulsion strength scales with urgency
						float repulsion_strength = urgency * max_repulsion_force_;
						
						total_repulsion[0] += repulsion_dir[0] * repulsion_strength;
						total_repulsion[1] += repulsion_dir[1] * repulsion_strength;
						total_repulsion[2] += repulsion_dir[2] * repulsion_strength;
					}
				}
				
				// Apply altitude separation if needed
				if (current_distance < min_separation_distance_ * 0.8f) {
					if (drone_id < other_id) {
						// This drone goes down
						total_repulsion[2] -= altitude_separation_distance_ * urgency;
					} else {
						// This drone goes up
						total_repulsion[2] += altitude_separation_distance_ * urgency;
					}
				}
				
				// Log collision warnings
				if (urgency > 0.5f) {
					RCLCPP_WARN(this->get_logger(), 
						"COLLISION THREAT: drone %d <-> drone %d | "
						"Dist: %.2fm -> %.2fm | Closure: %.2fm/s | Urgency: %.2f",
						drone_id, other_id, current_distance, predicted_distance, 
						closure_rate, urgency);
				}
			}
		}
		
		// Limit total repulsion to max force
		float total_repulsion_magnitude = std::sqrt(
			total_repulsion[0] * total_repulsion[0] + 
			total_repulsion[1] * total_repulsion[1] + 
			total_repulsion[2] * total_repulsion[2]
		);
		
		if (total_repulsion_magnitude > max_repulsion_force_) {
			float scale = max_repulsion_force_ / total_repulsion_magnitude;
			total_repulsion[0] *= scale;
			total_repulsion[1] *= scale;
			total_repulsion[2] *= scale;
		}
		
		// Apply repulsion to target
		adjusted_target[0] += total_repulsion[0];
		adjusted_target[1] += total_repulsion[1];
		adjusted_target[2] += total_repulsion[2];
		
		return adjusted_target;
	}
	
	/**
	 * @brief Timer callback that handles all drones
	 *        This runs every 100ms and processes all drones
	 */
	void timer_callback()
	{
		// Calculate elapsed time since start
		double elapsed = (this->get_clock()->now() - start_time_).seconds();
		double formation_delay = 10.0;  // 10 second delay before formation
		
		// Process each drone
		for (uint8_t i = 0; i < num_drones_; i++) {
			// After 10 setpoints (1 second), switch to offboard mode and arm
			if (setpoint_counters_[i] == 10) {
				// Change to Offboard mode
				publish_vehicle_command(i, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				in_offboard_mode_[i] = true;
				
				// Arm the vehicle
				arm(i);
			}
			
			// Always send offboard control mode and trajectory setpoint
			// This is required to keep the vehicle in offboard mode
			publish_offboard_control_mode(i);
			
			// Calculate desired world position
			std::array<float, 3> world_pos;
			
			// Get spawn offset for this drone
			std::array<float, 3> spawn_offset;
			{
				std::lock_guard<std::mutex> lock(spawn_offset_mutex_);
				spawn_offset = spawn_offsets_[i];
			}

			if (elapsed < formation_delay) {
				// Before delay: all drones hover at their spawn positions
				world_pos = {
					spawn_offset[0],
					spawn_offset[1],
					-5.0f  // Altitude
				};
			} else {
				// After delay: start formation
				if (i == 0) {
					// Leader: use current position (no prediction, no collision avoidance)
					// Leader follows its predetermined path without interference
					std::lock_guard<std::mutex> lock(leader_pos_mutex_);
					if (leader_position_received_) {
						world_pos = leader_world_pos_;
					} else {
						// Fallback to origin if not received yet
						world_pos = {0.0f, 0.0f, -5.0f};
					}
				} else {
					// Followers: predict leader position and apply collision avoidance
					std::array<float, 3> leader_world;
					{
						std::lock_guard<std::mutex> lock(leader_pos_mutex_);
						if (leader_position_received_) {
							if (use_predictive_control_) {
								// Followers predict leader's future position for smoother following
								leader_world = {
									leader_world_pos_[0] + leader_velocity_[0] * prediction_time_,
									leader_world_pos_[1] + leader_velocity_[1] * prediction_time_,
									leader_world_pos_[2] + leader_velocity_[2] * prediction_time_
								};
							} else {
								leader_world = leader_world_pos_;
							}
						} else {
							// Fallback to origin if not received yet
							leader_world = {0.0f, 0.0f, -5.0f};
						}
					}
					
					// Calculate follower positions relative to predicted leader position
					world_pos = {
						leader_world[0] + formation_offsets_[i][0],
						leader_world[1] + formation_offsets_[i][1],
						leader_world[2] + formation_offsets_[i][2]
					};
					
					// Apply collision avoidance to followers only (leader stays on path)
					if (collision_avoidance_enabled_) {
						world_pos = apply_collision_avoidance(i, world_pos);
					}
				}
			}
			
			// Determine yaw (all drones match the leader's yaw)
			float yaw = 0.0f;
			if (elapsed >= formation_delay) {
				std::lock_guard<std::mutex> lock(leader_yaw_mutex_);
				if (leader_attitude_received_) {
					yaw = leader_yaw_;
				}
			}

			// Calculate desired velocity for followers
			std::array<float, 3> desired_velocity = {
				std::numeric_limits<float>::quiet_NaN(),
				std::numeric_limits<float>::quiet_NaN(),
				std::numeric_limits<float>::quiet_NaN()
			};
			
			if (i > 0 && elapsed >= formation_delay && enable_follower_velocity_control_) {
				// Get leader velocity
				std::lock_guard<std::mutex> lock(leader_pos_mutex_);
				if (leader_position_received_) {
					// Scale leader velocity by multiplier
					desired_velocity = {
						leader_velocity_[0] * follower_velocity_multiplier_,
						leader_velocity_[1] * follower_velocity_multiplier_,
						leader_velocity_[2] * follower_velocity_multiplier_
					};
				}
			}

			// Convert to local setpoint (subtract spawn offset)
			publish_trajectory_setpoint(i, {
				world_pos[0] - spawn_offset[0],
				world_pos[1] - spawn_offset[1],
				world_pos[2] - spawn_offset[2]
			}, desired_velocity, yaw);
			
			// Increment counter (stop after reaching 11)
			if (setpoint_counters_[i] < 11) {
				setpoint_counters_[i]++;
			}
		}
	}
	
	/**
	 * @brief Publish offboard control mode for a specific drone
	 * @param drone_id The drone index (0 to num_drones-1)
	 */
	void publish_offboard_control_mode(uint8_t drone_id)
	{
		OffboardControlMode msg{};
		msg.position = true;
		// Enable velocity control for followers if enabled
		msg.velocity = (drone_id > 0 && enable_follower_velocity_control_);
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publishers_[drone_id]->publish(msg);
	}
	
	/**
	 * @brief Publish trajectory setpoint for a specific drone
	 * @param drone_id The drone index (0 to num_drones-1)
	 * @param position Local position [x, y, z] relative to spawn position
	 * @param velocity Local velocity [vx, vy, vz] (NaN if not controlled)
	 */
	void publish_trajectory_setpoint(uint8_t drone_id, const std::array<float, 3>& position, 
	                                const std::array<float, 3>& velocity, float yaw)
	{
		TrajectorySetpoint msg{};
		msg.position = position;
		msg.velocity = velocity;
		msg.yaw = yaw;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publishers_[drone_id]->publish(msg);
	}
	
	/**
	 * @brief Publish vehicle command for a specific drone
	 * @param drone_id The drone index (0 to num_drones-1)
	 * @param command Command code
	 * @param param1 Command parameter 1
	 * @param param2 Command parameter 2
	 */
	void publish_vehicle_command(uint8_t drone_id, uint16_t command, float param1, float param2)
	{
		VehicleCommand msg{};
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		// Map drone_id to instance_id (instances start from 1)
		// According to PX4 multi-vehicle docs: target_system = instance_id + 1
		uint8_t instance_id = drone_id + 1;
		msg.target_system = instance_id + 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		vehicle_command_publishers_[drone_id]->publish(msg);
	}
	
	/**
	 * @brief Arm a specific drone
	 * @param drone_id The drone index (0 to num_drones-1)
	 */
	void arm(uint8_t drone_id)
	{
		publish_vehicle_command(drone_id, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
		armed_[drone_id] = true;
		uint8_t instance_id = drone_id + 1;
		uint8_t target_system = instance_id + 1;
		RCLCPP_INFO(this->get_logger(), "Arm command sent to drone %d (instance %d, target_system %d)", 
			drone_id, instance_id, target_system);
	}
	
	/**
	 * @brief Disarm a specific drone
	 * @param drone_id The drone index (0 to num_drones-1)
	 */
	void disarm(uint8_t drone_id)
	{
		publish_vehicle_command(drone_id, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
		armed_[drone_id] = false;
		RCLCPP_INFO(this->get_logger(), "Disarm command sent to drone %d", drone_id);
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting multi-vehicle offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	
	// Parse command-line arguments for number of drones
	uint8_t num_drones = 5;  // Default to 5 drones
	
	// Parse arguments: --num_drones 5
	for (int i = 1; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--num_drones" && i + 1 < argc) {
			num_drones = static_cast<uint8_t>(std::stoi(argv[++i]));
		}
	}
	
	std::cout << "Controlling " << static_cast<int>(num_drones) << " drones" << std::endl;
	
	rclcpp::spin(std::make_shared<MultiVehicleOffboardControl>(num_drones));
	
	rclcpp::shutdown();
	return 0;
}

