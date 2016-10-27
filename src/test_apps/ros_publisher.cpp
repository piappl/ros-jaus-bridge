#include <ros/ros.h>
#include <rj_bridge_ros_msgs/SetWrenchEffort.h>

#include <functional>
#include <iostream>
#include <random>

int main(int argc, char **argv)
{
    std::default_random_engine engine{};
    std::uniform_int_distribution<> distribution{0, 100};
    auto randomize = std::bind(distribution, engine);

    ros::init(argc, argv, "ros_publisher");

    ros::NodeHandle node_handle;

    std::cout << "ROS topic: ";
    std::string topic{};
    std::cin >> topic;

    ros::Publisher publisher = node_handle.advertise
            <rj_bridge_ros_msgs::SetWrenchEffort>(topic, 1000);

    ros::Rate loop_rate{10};

    while (ros::ok())
    {
        rj_bridge_ros_msgs::SetWrenchEffort msg;

        msg.PropulsiveLinearEffortX = randomize();
        msg.PropulsiveLinearEffortY = randomize();
        msg.PropulsiveLinearEffortZ = randomize();

        ROS_INFO("Publishing message on topic: %s ...", topic.c_str());

        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}