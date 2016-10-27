#include "wrench_effort_plugin.h"

PluginType WrenchEffortPlugin::getType()
{
    return translateType<rj_bridge_ros_msgs::SetWrenchEffort,
            openjaus::mobility::SetWrenchEffort>();
}

std::string WrenchEffortPlugin::getMaintainer() const
{
    return std::string{"PIAP_PL"};
}

QVariant WrenchEffortPlugin::translateRosMsg(QVariant msg) const
{
    rj_bridge_ros_msgs::SetWrenchEffort ros_msg =
            msg.value<rj_bridge_ros_msgs::SetWrenchEffort>();
    openjaus::mobility::SetWrenchEffort jaus_msg{};

    jaus_msg.enablePropulsiveLinearEffortX();
    jaus_msg.enablePropulsiveLinearEffortY();
    jaus_msg.enablePropulsiveLinearEffortZ();
    jaus_msg.enablePropulsiveRotationalEffortX();
    jaus_msg.enablePropulsiveRotationalEffortY();
    jaus_msg.enablePropulsiveRotationalEffortZ();
    jaus_msg.enableResistiveLinearEffortX();
    jaus_msg.enableResistiveLinearEffortY();
    jaus_msg.enableResistiveLinearEffortZ();
    jaus_msg.enableResistiveRotationalEffortX();
    jaus_msg.enableResistiveRotationalEffortY();
    jaus_msg.enableResistiveRotationalEffortZ();

    jaus_msg.setPropulsiveLinearEffortX_percent(
                ros_msg.PropulsiveLinearEffortX);
    jaus_msg.setPropulsiveLinearEffortY_percent(
                ros_msg.PropulsiveLinearEffortY);
    jaus_msg.setPropulsiveLinearEffortZ_percent(
                ros_msg.PropulsiveLinearEffortZ);
    jaus_msg.setPropulsiveRotationalEffortX_percent(
                ros_msg.PropulsiveRotationalEffortX);
    jaus_msg.setPropulsiveRotationalEffortY_percent(
                ros_msg.PropulsiveRotationalEffortY);
    jaus_msg.setPropulsiveRotationalEffortZ_percent(
                ros_msg.PropulsiveRotationalEffortZ);
    jaus_msg.setResistiveLinearEffortX_percent(
                ros_msg.ResistiveLinearEffortX);
    jaus_msg.setResistiveLinearEffortY_percent(
                ros_msg.ResistiveLinearEffortY);
    jaus_msg.setResistiveLinearEffortZ_percent(
                ros_msg.ResistiveLinearEffortZ);
    jaus_msg.setResistiveRotationalEffortX_percent(
                ros_msg.ResistiveRotationalEffortX);
    jaus_msg.setResistiveRotationalEffortY_percent(
                ros_msg.ResistiveRotationalEffortY);
    jaus_msg.setResistiveRotationalEffortZ_percent(
                ros_msg.ResistiveRotationalEffortZ);

    QVariant result;
    result.setValue(jaus_msg);
    return result;
}

QVariant WrenchEffortPlugin::translateJausMsg(QVariant msg) const
{
    openjaus::mobility::SetWrenchEffort jaus_msg =
            msg.value<openjaus::mobility::SetWrenchEffort>();
    rj_bridge_ros_msgs::SetWrenchEffort ros_msg{};

    ros_msg.PropulsiveLinearEffortX =
            jaus_msg.getPropulsiveLinearEffortX_percent();
    ros_msg.PropulsiveLinearEffortY =
            jaus_msg.getPropulsiveLinearEffortY_percent();
    ros_msg.PropulsiveLinearEffortZ =
            jaus_msg.getPropulsiveLinearEffortZ_percent();
    ros_msg.PropulsiveRotationalEffortX =
            jaus_msg.getPropulsiveRotationalEffortX_percent();
    ros_msg.PropulsiveRotationalEffortY =
            jaus_msg.getPropulsiveRotationalEffortY_percent();
    ros_msg.PropulsiveRotationalEffortZ =
            jaus_msg.getPropulsiveRotationalEffortZ_percent();
    ros_msg.ResistiveLinearEffortX =
            jaus_msg.getResistiveLinearEffortX_percent();
    ros_msg.ResistiveLinearEffortY =
            jaus_msg.getResistiveLinearEffortY_percent();
    ros_msg.ResistiveLinearEffortZ =
            jaus_msg.getResistiveLinearEffortZ_percent();
    ros_msg.ResistiveRotationalEffortX =
            jaus_msg.getResistiveRotationalEffortX_percent();
    ros_msg.ResistiveRotationalEffortY =
            jaus_msg.getResistiveRotationalEffortY_percent();
    ros_msg.ResistiveRotationalEffortZ =
            jaus_msg.getResistiveRotationalEffortZ_percent();

    QVariant result;
    result.setValue(ros_msg);
    return result;
}