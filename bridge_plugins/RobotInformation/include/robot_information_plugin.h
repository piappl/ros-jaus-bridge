#ifndef ROBOT_INFORMATION_PLUGIN_H
#define ROBOT_INFORMATION_PLUGIN_H

#include <rj_bridge_ros_msgs/RobotInformation.h>
#include "jaus_msg/robot_information.h"

#include "../plugin_interface/plugin_interface.h"

Q_DECLARE_METATYPE(openjaus::RobotInformation)
Q_DECLARE_METATYPE(rj_bridge_ros_msgs::RobotInformation)

class RobotInformationPlugin : public QObject, public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "rj_bridge.PluginInterface" FILE "RobotInformation.json")
    Q_INTERFACES(PluginInterface)

public:
    PluginType getType() override;
    QVariant translateJausMsg(QVariant msg) const override;
    QVariant translateRosMsg(QVariant msg) const override;
    std::string getMaintainer() const override;
};

#endif // ROBOT_INFORMATION_PLUGIN_H