#include <iostream>

#include <rj_bridge_ros_msgs/SetWrenchEffort.h>
#include <openjaus/mobility.h>

#include <geometry_msgs/Pose.h>

#include "bridge_core.h"

using namespace rj_bridge;

Q_DECLARE_METATYPE(openjaus::mobility::SetWrenchEffort)
Q_DECLARE_METATYPE(rj_bridge_ros_msgs::SetWrenchEffort)

Q_DECLARE_METATYPE(geometry_msgs::Pose)
Q_DECLARE_METATYPE(openjaus::mobility::SetLocalPose)

int main(int argc, char **argv)
{
    try
    {
        BridgeCore bridge_core(argc, argv, "ros_jaus_bridge",
                               "/home/stabaka/Projects/rj_bridge_plugins/plugins");
        bridge_core.createBridgeJ2R<openjaus::mobility::SetWrenchEffort,
                rj_bridge_ros_msgs::SetWrenchEffort>("wrench_effort_ros", "wrench_effort_out");
        bridge_core.createBridgeR2J<rj_bridge_ros_msgs::SetWrenchEffort,
                openjaus::mobility::SetWrenchEffort>("wrench_effort_in", "jaus_cmp");

        bridge_core.createBridgeR2J<geometry_msgs::Pose,
                openjaus::mobility::SetLocalPose>("local_pose", "jaus_cmp_local_pose");

        bridge_core.run(10);
    }
    catch (const Exception &ex)
    {
        std::cout << ex.what() << std::endl;
        return -1;
    }

    return 0;
}