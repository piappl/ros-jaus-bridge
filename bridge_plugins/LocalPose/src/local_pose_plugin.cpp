#include "local_pose_plugin.h"

PluginType WrenchEffortPlugin::getType()
{
    return translateType<geometry_msgs::Pose,
            openjaus::mobility::SetLocalPose>();
}

std::string WrenchEffortPlugin::getMaintainer() const
{
    return std::string{"PIAP_PL"};
}

QVariant WrenchEffortPlugin::translateRosMsg(QVariant msg) const
{
    geometry_msgs::Pose ros_msg = msg.value<geometry_msgs::Pose>();
    openjaus::mobility::SetLocalPose jaus_msg{};

    jaus_msg.enableX();
    jaus_msg.enableY();
    jaus_msg.enableZ();
    jaus_msg.enableRoll();
    jaus_msg.enablePitch();
    jaus_msg.enableYaw();

    jaus_msg.setX_m(ros_msg.position.x);
    jaus_msg.setY_m(ros_msg.position.y);
    jaus_msg.setZ_m(ros_msg.position.z);

    tf::Quaternion quat(ros_msg.orientation.x, ros_msg.orientation.y,
                        ros_msg.orientation.z, ros_msg.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    jaus_msg.setRoll_rad(roll);
    jaus_msg.setPitch_rad(pitch);
    jaus_msg.setYaw_rad(yaw);

    QVariant result;
    result.setValue(jaus_msg);
    return result;
}

QVariant WrenchEffortPlugin::translateJausMsg(QVariant msg) const
{
    openjaus::mobility::SetLocalPose jaus_msg =
            msg.value<openjaus::mobility::SetLocalPose>();
    geometry_msgs::Pose ros_msg{};

    ros_msg.position.x = jaus_msg.getX_m();
    ros_msg.position.y = jaus_msg.getY_m();
    ros_msg.position.z = jaus_msg.getZ_m();

    tf::Quaternion quat(jaus_msg.getYaw_rad(), jaus_msg.getPitch_rad(),
                        jaus_msg.getRoll_rad());

    ros_msg.orientation.x = quat.x();
    ros_msg.orientation.y = quat.y();
    ros_msg.orientation.z = quat.z();
    ros_msg.orientation.w = quat.w();

    QVariant result;
    result.setValue(ros_msg);
    return result;
}