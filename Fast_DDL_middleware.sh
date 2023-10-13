 #! /bin/sh


 # creating a package
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp


# creating src/sync_async_writer.cpp file with the content below

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SyncAsyncPublisher : public rclcpp::Node
{
public:
    SyncAsyncPublisher()
        : Node("sync_async_publisher"), count_(0)
    {
        // Create the synchronous publisher on topic 'sync_topic'
        sync_publisher_ = this->create_publisher<std_msgs::msg::String>("sync_topic", 10);

        // Create the asynchronous publisher on topic 'async_topic'
        async_publisher_ = this->create_publisher<std_msgs::msg::String>("async_topic", 10);

        // This timer will trigger the publication of new data every half a second
        timer_ = this->create_wall_timer(
                500ms, std::bind(&SyncAsyncPublisher::timer_callback, this));
    }

private:
    /**
     * Actions to run every time the timer expires
     */
    void timer_callback()
    {
        // Create a new message to be sent
        auto sync_message = std_msgs::msg::String();
        sync_message.data = "SYNC: Hello, world! " + std::to_string(count_);

        // Log the message to the console to show progress
        RCLCPP_INFO(this->get_logger(), "Synchronously publishing: '%s'", sync_message.data.c_str());

        // Publish the message using the synchronous publisher
        sync_publisher_->publish(sync_message);

        // Create a new message to be sent
        auto async_message = std_msgs::msg::String();
        async_message.data = "ASYNC: Hello, world! " + std::to_string(count_);

        // Log the message to the console to show progress
        RCLCPP_INFO(this->get_logger(), "Asynchronously publishing: '%s'", async_message.data.c_str());

        // Publish the message using the asynchronous publisher
        async_publisher_->publish(async_message);

        // Prepare the count for the next message
        count_++;
    }

    // This timer will trigger the publication of new data every half a second
    rclcpp::TimerBase::SharedPtr timer_;

    // A publisher that publishes asynchronously
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr async_publisher_;

    // A publisher that publishes synchronously
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_publisher_;

    // Number of messages sent so far
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncAsyncPublisher>());
    rclcpp::shutdown();
    return 0;
}





# adding executables to CMakeList.txt
add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

install(TARGETS
    SyncAsyncWriter
    DESTINATION lib/${PROJECT_NAME})

# cleaning up the file to this:
cmake_minimum_required(VERSION 3.8)
project(sync_async_node_example_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(SyncAsyncWriter src/sync_async_writer.cpp)
ament_target_dependencies(SyncAsyncWriter rclcpp std_msgs)

install(TARGETS
    SyncAsyncWriter
    DESTINATION lib/${PROJECT_NAME})

ament_package()




# creating SyncAsync.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- default publisher profile -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </publisher>

    <!-- default subscriber profile -->
    <subscriber profile_name="default_subscriber" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </subscriber>

    <!-- publisher profile for topic sync_topic -->
    <publisher profile_name="/sync_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>SYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

    <!-- publisher profile for topic async_topic -->
    <publisher profile_name="/async_topic">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

 </profiles>








 # running publisher node

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml

source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncWriter




# initializing a node with subscribers

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SyncAsyncSubscriber : public rclcpp::Node
{
public:

    SyncAsyncSubscriber()
        : Node("sync_async_subscriber")
    {
        // Create the synchronous subscriber on topic 'sync_topic'
        // and tie it to the topic_callback
        sync_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "sync_topic", 10, std::bind(&SyncAsyncSubscriber::topic_callback, this, _1));

        // Create the asynchronous subscriber on topic 'async_topic'
        // and tie it to the topic_callback
        async_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "async_topic", 10, std::bind(&SyncAsyncSubscriber::topic_callback, this, _1));
    }

private:

    /**
     * Actions to run every time a new message is received
     */
    void topic_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    // A subscriber that listens to topic 'sync_topic'
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sync_subscription_;

    // A subscriber that listens to topic 'async_topic'
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr async_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SyncAsyncSubscriber>());
    rclcpp::shutdown();
    return 0;
}



# adding SyncAsyncReader executable to CMakeList
add_executable(SyncAsyncReader src/sync_async_reader.cpp)
ament_target_dependencies(SyncAsyncReader rclcpp std_msgs)

install(TARGETS
    SyncAsyncReader
    DESTINATION lib/${PROJECT_NAME})


# running subscriber node

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/SyncAsync.xml


source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncReader


# setting up a service and a client

# writing  src/ping_service.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"

/**
 * Service action: responds with success=true and prints the request on the console
 */
void ping(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
{
    // The request data is unused
    (void) request;

    // Build the response
    response->success = true;

    // Log to the console
    RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Incoming request");
    RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Sending back response");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node and the service
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ping_server");
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service =
        node->create_service<example_interfaces::srv::Trigger>("ping", &ping);

    // Log that the service is ready
    RCLCPP_INFO(rclcpp::get_logger("ping_server"), "Ready to serve.");

    // run the node
    rclcpp::spin(node);
    rclcpp::shutdown();
}


# writing a scr/ping_client.cpp

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/trigger.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node and the client
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ping_client");
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr client =
        node->create_client<example_interfaces::srv::Trigger>("ping");

    // Create a request
    auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();

    // Wait for the service to be available
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("ping_client"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Service not available, waiting again...");
    }

    // Now that the service is available, send the request
    RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Sending request");
    auto result = client->async_send_request(request);

    // Wait for the result and log it to the console
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("ping_client"), "Response received");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ping_client"), "Failed to call service ping");
    }

    rclcpp::shutdown();
    return 0;
}


# adding ping_servie and ping_client executables to the CMakeList.txt
find_package(example_interfaces REQUIRED)

add_executable(ping_service src/ping_service.cpp)
ament_target_dependencies(ping_service example_interfaces rclcpp)

add_executable(ping_client src/ping_client.cpp)
ament_target_dependencies(ping_client example_interfaces rclcpp)

install(TARGETS
    ping_service
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    ping_client
    DESTINATION lib/${PROJECT_NAME})


# creating ping.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- default publisher profile -->
    <publisher profile_name="default_publisher" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </publisher>

    <!-- default subscriber profile -->
    <subscriber profile_name="default_subscriber" is_default_profile="true">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
    </subscriber>

    <!-- service publisher is SYNC -->
    <publisher profile_name="service">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>SYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

    <!-- client publisher is ASYNC -->
    <publisher profile_name="client">
        <historyMemoryPolicy>DYNAMIC</historyMemoryPolicy>
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
            </publishMode>
        </qos>
    </publisher>

</profiles>






# running the nodes
# sourcing the setup files in two different terminals
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=path/to/ping.xml


# in the first terminal
ros2 run sync_async_node_example_cpp ping_service

# in the second terminal
ros2 run sync_async_node_example_cpp ping_client



