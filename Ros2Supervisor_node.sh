#! /bin/sh


# executing ros2_supervisor in WebotLauncher
webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'worlds', world]),
    mode=mode,
    ros2_supervisor=True
)


# importing Webots node
ros2 service call /Ros2Supervisor/spawn_node_from_string webots_ros2_msgs/srv/SpawnNodeFromString "data: Robot { name \"imported_robot\" }"


# omitting Webots node
ros2 topic pub --once /Ros2Supervisor/remove_node std_msgs/msg/String "{data: imported_robot}"

# recording animations
ros2 service call /Ros2Supervisor/animation_start_recording webots_ros2_msgs/srv/SetString "{value: "<ABSOLUTE_PATH>/index.html"}"

ros2 service call /Ros2Supervisor/animation_stop_recording webots_ros2_msgs/srv/GetBool "{ask: True}"

