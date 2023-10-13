 #! /bin/sh


 # in a new terminal write:
 fastdds discovery --server-id 0

 # in a new terminal write:
 export ROS_DISCOVERY_SERVER=127.0.0.1:11811

 ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server


 # launching talker node file
 export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server


# openning a new terminal and running a new listener
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

# in another terminal
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener



# in another terminal
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener




# Discovery partitions
# in another terminal
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811

fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2

export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2




# Daemon's related tools
fastdds discovery -i 0 -l 127.0.0.1 -p 11811

# running a talker and listener files
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

# starting a ROS2 DAEMON using Super Client
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter



# rqt grpah
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph



# no Daemon tools

# running a server
fastdds discovery -i 0 -l 127.0.0.1 -p 11811

# running talker and listener
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker


export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 topic list --no-daemon
ros2 node info /talker --no-daemon --spin-time 2



