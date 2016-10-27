#ifndef WRENCH_EFFORT_PLUGIN_H
#define WRENCH_EFFORT_PLUGIN_H

#include <openjaus/mobility.h>
#include <rj_bridge_ros_msgs/SetWrenchEffort.h>

#include "../plugin_interface/plugin_interface.h"

Q_DECLARE_METATYPE(openjaus::mobility::SetWrenchEffort)
Q_DECLARE_METATYPE(rj_bridge_ros_msgs::SetWrenchEffort)

class WrenchEffortPlugin : public QObject, public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "rj_bridge.PluginInterface" FILE "WrenchEffort.json")
    Q_INTERFACES(PluginInterface)

public:
    PluginType getType() override;
    QVariant translateJausMsg(QVariant msg) const override;
    QVariant translateRosMsg(QVariant msg) const override;
    std::string getMaintainer() const override;
};

#endif // WRENCH_EFFORT_PLUGIN_H