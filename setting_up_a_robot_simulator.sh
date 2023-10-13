#! /bin/sh



# updating my_robot.urdf
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <device reference="ds0" type="DistanceSensor">
            <ros>
                <topicName>/left_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <device reference="ds1" type="DistanceSensor">
            <ros>
                <topicName>/right_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <plugin type="my_robot_driver::MyRobotDriver" />
    </webots>
</robot>



# building a ROS node to avoid obstacles

#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class ObstacleAvoider : public rclcpp::Node {
public:
  explicit ObstacleAvoider();

private:
  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

  double left_sensor_value{0.0};
  double right_sensor_value{0.0};
};




# navigating to my_package/src and creating a file ObstacleAvoider.cpp
#include "my_package/ObstacleAvoider.hpp"

#define MAX_RANGE 0.15

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      std::bind(&ObstacleAvoider::leftSensorCallback, this,
                std::placeholders::_1));

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      std::bind(&ObstacleAvoider::rightSensorCallback, this,
                std::placeholders::_1));
}

void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = 0.1;

  if (left_sensor_value < 0.9 * MAX_RANGE ||
      right_sensor_value < 0.9 * MAX_RANGE) {
    command_message->angular.z = -2.0;
  }

  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}




# updating additional files

# in CMakeLists.txt 
cmake_minimum_required(VERSION 3.5)
project(my_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# Obstacle avoider
include_directories(
  include
)
add_executable(obstacle_avoider
  src/ObstacleAvoider.cpp
)
ament_target_dependencies(obstacle_avoider
  rclcpp
  geometry_msgs
  sensor_msgs
)
install(TARGETS
  obstacle_avoider
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()



# replace def generate_launch_description(): in robot_launch.py 
def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])



# testing obstacle avoidance
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py

