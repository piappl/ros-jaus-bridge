#include <ros/ros.h>
#include <rj_bridge_ros_msgs/SetWrenchEffort.h>

#include <iostream>

void rosCallback(const rj_bridge_ros_msgs::SetWrenchEffort::ConstPtr &msg)
{
    ROS_INFO("Received ROS message SetWrenchEffort with data [%d, %d, %d].",
             msg->PropulsiveLinearEffortX, msg->PropulsiveLinearEffortY,
             msg->PropulsiveLinearEffortZ);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_subscriber");

    std::cout << "ROS topic: ";
    std::string topic{};
    std::cin >> topic;

    ros::NodeHandle node_handle;
    ros::Subscriber subscriber = node_handle.subscribe(topic, 1000, rosCallback);

    ros::spin();

    return 0;
}