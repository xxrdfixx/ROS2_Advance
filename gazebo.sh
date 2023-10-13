#! /bin/sh

# opening simulation
ign gazebo -v 4 -r visualize_lidar.sdf

ign topic -l

ros2 topic list

# setting up ros2

sudo apt-get install ros-humble-ros-ign-bridge
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"
sudo apt-get install ros-humble-teleop-twist-keyboard
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel



# visualizing lidar data in ROS2
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan

source /opt/ros/humble/setup.bash
rviz2
