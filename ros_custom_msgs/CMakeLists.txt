cmake_minimum_required(VERSION 2.8.3)
project(rj_bridge_ros_msgs)

find_package(catkin REQUIRED roscpp genmsg message_generation)

add_message_files(
    FILES
    SetWrenchEffort.msg
    RobotInformation.msg
    # Add more here
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)