#! /bin/sh


# adding respawn parameter
def generate_launch_description():
    robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path}
        ],

        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    # Starts Webots
    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

    return LaunchDescription([
        webots,
        robot_driver
    ])


# reapplying Handler for Multiple Nodes
def get_ros2_control_spawners(*args):
    # Declare here all nodes that must be restarted at simulation reset
    ros_control_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller']
    )
    return [
        ros_control_node
    ]

def generate_launch_description():
    robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path}
        ],

        # Every time one resets the simulation the controller is automatically respawned
        respawn=True
    )

    # Starts Webots
    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

    # Declare the reset handler that respawns nodes when robot_driver exits
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=robot_driver,
            on_exit=get_ros2_control_spawners,
        )
    )

    return LaunchDescription([
        webots,
        robot_driver,
        reset_handler
    ] + get_ros2_control_spawners())



# executing file without other nodes
def generate_launch_description():
    # Starts Webots
    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]))

    return LaunchDescription([
        webots
    ])


# second execution
def generate_launch_description():
    robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path}
        ]
    )

    ros_control_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller']
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', nav2_map),
            ('params_file', nav2_params),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # Declare the handler that shuts all nodes down when robot_driver exits
    shutdown_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=robot_driver,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        robot_driver,
        ros_control_node,
        nav2_node,
        rviz,
        shutdown_handler
    ])
    
