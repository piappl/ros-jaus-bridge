#include "robot_information_plugin.h"

PluginType RobotInformationPlugin::getType()
{
    return translateType<rj_bridge_ros_msgs::RobotInformation,
            openjaus::RobotInformation>();
}

std::string RobotInformationPlugin::getMaintainer() const
{
    return std::string{"PIAP_PL"};
}

QVariant RobotInformationPlugin::translateRosMsg(QVariant msg) const
{
    rj_bridge_ros_msgs::RobotInformation ros_msg =
            msg.value<rj_bridge_ros_msgs::RobotInformation>();
    openjaus::RobotInformation jaus_msg{};

    jaus_msg.setRobotName(ros_msg.robot_name);
    jaus_msg.setRobotID(ros_msg.robot_id);
    jaus_msg.setRobotType(ros_msg.robot_type);
    jaus_msg.setLocalizationType(ros_msg.localization_type);
    jaus_msg.setX(ros_msg.x);
    jaus_msg.setY(ros_msg.y);
    jaus_msg.setTheta(ros_msg.theta);
    jaus_msg.setRobotDescription(ros_msg.description);

    QVariant result;
    result.setValue(jaus_msg);
    return result;
}

QVariant RobotInformationPlugin::translateJausMsg(QVariant msg) const
{
    openjaus::RobotInformation jaus_msg =
            msg.value<openjaus::RobotInformation>();
    rj_bridge_ros_msgs::RobotInformation ros_msg{};

    ros_msg.robot_name = jaus_msg.getRobotName();
    ros_msg.robot_id = jaus_msg.getRobotID();
    ros_msg.robot_type = jaus_msg.getRobotType();
    ros_msg.localization_type = jaus_msg.getLocalizationType();
    ros_msg.x = jaus_msg.getX();
    ros_msg.y = jaus_msg.getY();
    ros_msg.theta = jaus_msg.getTheta();
    ros_msg.description = jaus_msg.getRobotDescription();

    QVariant result;
    result.setValue(ros_msg);
    return result;
}