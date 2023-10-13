#! /bin/sh



# installing webots_ros2
sudo apt-get install ros-humble-webots-ros2


# executing webots_ros2_universal_robot

# sourcing the environmnet
source /opt/ros/humble/setup.bash

export WEBOTS_HOME=/usr/local/webotscd ~/ros2_ws
source install/local_setup.bash


# starting demo packages

ros2 launch webots_ros2_universal_robot multirobot_launch.py
