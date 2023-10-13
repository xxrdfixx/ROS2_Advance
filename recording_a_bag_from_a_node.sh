#! /bin/sh

# building a package
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs

# updating package.xml
<description>C++ bag writing tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>


# writing C++ node
# writing simple_bag_recorder.cpp inside ros2_ws/src/bag_recorder_nodes/src

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("my_bag");

    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}





# adding executable to CMakeList.txt
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp std_msgs)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)




# build and run
colcon build --packages-select bag_recorder_nodes

# in a new terminal, navigate to ros2_ws and run:
source install/setup.bash

# running the node
ros2 run bag_recorder_nodes simple_bag_recorder


# in a new terminal
ros2 run demo_nodes_cpp talker


# after running both nodes
ros2 run demo_nodes_cpp listener
# in a second terminal
ros2 bag play my_bag





# recroding a C++ node and building data_generator_node.cpp inside ros2_ws/src/bag_recorder_nodes/src

#include <chrono>
#include <example_interfaces/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

using namespace std::chrono_literals;

class DataGenerator : public rclcpp::Node
{
public:
  DataGenerator()
  : Node("data_generator")
  {
    data_.data = 0;
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("timed_synthetic_bag");

    writer_->create_topic(
      {"synthetic",
       "example_interfaces/msg/Int32",
       rmw_get_serialization_format(),
       ""});

    timer_ = create_wall_timer(1s, std::bind(&DataGenerator::timer_callback, this));
  }

private:
  void timer_callback()
  {
    writer_->write(data_, "synthetic", now());

    ++data_.data;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  example_interfaces::msg::Int32 data_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataGenerator>());
  rclcpp::shutdown();
  return 0;
}


# addding executable to CMakeList.txt
add_executable(data_generator_node src/data_generator_node.cpp)
ament_target_dependencies(data_generator_node rclcpp rosbag2_cpp example_interfaces)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)



# build and run in ros2_ws
colcon build --packages-select bag_recorder_nodes

# in a another terminal
ros2 run bag_recorder_nodes data_generator_node


# after 30 seconds stop and play back
ros2 bag play timed_synthetic_bag
# in a new terminal
ros2 topic echo /synthetic



# recording synthetic data from an executable
# navigate to ros2_ws/src/bag_recorder_nodes/src and create data_generator_executable.cpp

#include <chrono>
#include <rclcpp/rclcpp.hpp>  // For rclcpp::Clock, rclcpp::Duration and rclcpp::Time
#include <example_interfaces/msg/int32.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using namespace std::chrono_literals;

int main(int, char**)
{
  example_interfaces::msg::Int32 data;
  data.data = 0;
  std::unique_ptr<rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();

  writer_->open("big_synthetic_bag");

  writer_->create_topic(
    {"synthetic",
     "example_interfaces/msg/Int32",
     rmw_get_serialization_format(),
     ""});

  rclcpp::Clock clock;
  rclcpp::Time time_stamp = clock.now();
  for (int32_t ii = 0; ii < 100; ++ii) {
    writer_->write(data, "synthetic", time_stamp);
    ++data.data;
    time_stamp += rclcpp::Duration(1s);
  }

  return 0;
}


# adding executable to CMakeLists.txt
add_executable(data_generator_executable src/data_generator_executable.cpp)
ament_target_dependencies(data_generator_executable rclcpp rosbag2_cpp example_interfaces)

install(TARGETS
  data_generator_executable
  DESTINATION lib/${PROJECT_NAME}
)

# build and run in ros2_ws
colcon build --packages-select bag_recorder_nodes

# in a new terminal
source install/setup.bash

# running executable
ros2 run bag_recorder_nodes data_generator_executable

# playing back the created bag
ros2 bag play big_synthetic_bag

# in another terminal
ros2 topic echo /synthetic







