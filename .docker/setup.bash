#!/bin/bash

# Set up the environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/dev_ws/install/setup.bash"
source "/usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
# dds
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"