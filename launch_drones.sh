#!/bin/bash

# Start Drone 1 in first tab
terminator -T "Drone 1" \
    -e "bash -c '
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
exec bash'" &

sleep 6

# The following use --new-tab to add tabs to the same window
terminator --new-tab -T "Drone 2" \
    -e "bash -c '
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"3,3\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
exec bash'" &

terminator --new-tab -T "Drone 3" \
    -e "bash -c '
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"-3,3\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3
exec bash'" &

terminator --new-tab -T "Drone 4" \
    -e "bash -c '
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"3,-3\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 4
exec bash'" &

terminator --new-tab -T "Drone 5" \
    -e "bash -c '
cd ~/ws_offboard_control
source install/local_setup.bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/
source install/setup.bash
cd ~/src/PX4-Autopilot/
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"-3,-3\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 5
exec bash'" &

terminator --new-tab -T "MicroXRCEAgent" \
    -e "bash -c '
MicroXRCEAgent udp4 -p 8888
exec bash'" &

echo "All drones and MicroXRCEAgent launched!"

