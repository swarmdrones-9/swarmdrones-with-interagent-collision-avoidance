#LAUNCH THE DRONES
cd ~/ws_offboard_control
./launch_drones.sh

#LAUNCH THE SWARM NODE

# Terminal 2 - offboard control  
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
ros2 run px4_ros_com multi_vehicle_offboard_control -- --num_drones 5

#build
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_ros_com
source install/local_setup.bash





#path launch
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/px4_ros_com/src/examples/offboard_py/path_controller.py --path square 


#visulization of the path
#path launch
cd ~/ws_offboard_control
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/px4_ros_com/src/examples/offboard_py/visualize_path.py square_2.0ms_20251202_120103.csv



# If CSV is in the plots folder
python3 src/px4_ros_com/src/examples/offboard_py/visualize_path.py plots/square_2.0ms_20251202_120103.csv/square_2.0ms_20251202_120103.csv

# Or if CSV is in current directory
python3 src/px4_ros_com/src/examples/offboard_py/visualize_path.py square_2.0ms_20251127_165620.csv













#build before you start when you change code
cd ~/ws_offboard_control && source /opt/ros/jazzy/setup.bash && colcon build --packages-select px4_ros_com && source install/local_setup.bash

# Terminal 1 - Drone 1 (instance 1)
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001  PX4_SIM_MODEL=gz_x500_lidar_2d ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 2 - Drone 2 (instance 2)
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,3" PX4_SIM_MODEL=gz_x500_lidar_2d ./build/px4_sitl_default/bin/px4 -i 2

# Terminal 3 - Drone 3 (instance 3)
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-3" PX4_SIM_MODEL=gz_x500_lidar_2d ./build/px4_sitl_default/bin/px4 -i 3

# Terminal 4 - Drone 4 (instance 4)
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6" PX4_SIM_MODEL=gz_x500_lidar_2d ./build/px4_sitl_default/bin/px4 -i 4

# Terminal 5 - Drone 5 (instance 4)
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,-6" PX4_SIM_MODEL=gz_x500_lidar_2d ./build/px4_sitl_default/bin/px4 -i 5

#terminal 6 launch MicroXRCEagent
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
MicroXRCEAgent udp4 -p 8888



# Terminal 7 - lidar bridge 
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
ros2 run px4_ros_com swarm_bridge.py --num-drones 5 --world default


# Terminal 8 - offboard control  
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
ros2 run px4_ros_com multi_vehicle_offboard_control -- --num_drones 5


#terminal 9 rviz launch
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
ros2 launch px4_ros_com swarm_viz.launch.py


cd ~/ws_offboard_control
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build




#LAUNCH THE DRONES
cd ~/ws_offboard_control
./launch_drones.sh

#LAUNCH THE SWARM NODE

# Terminal 2 - offboard control  
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
ros2 run px4_ros_com multi_vehicle_offboard_control -- --num_drones 5




#to kill everything
tmux kill-session -t drones; killall -9 px4 gzserver gzclient ruby gz MicroXRCEAgent; pkill -9 -f "PX4_SYS_AUTOSTART" 
