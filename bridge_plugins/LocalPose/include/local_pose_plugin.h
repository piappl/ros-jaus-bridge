#ifndef WRENCH_EFFORT_PLUGIN_H
#define WRENCH_EFFORT_PLUGIN_H

#include <openjaus/mobility.h>
#include <geometry_msgs/Pose.h>

#include <tf/tf.h>

#include "../plugin_interface/plugin_interface.h"

Q_DECLARE_METATYPE(openjaus::mobility::SetLocalPose)
Q_DECLARE_METATYPE(geometry_msgs::Pose)

class WrenchEffortPlugin : public QObject, public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "rj_bridge.PluginInterface" FILE "LocalPose.json")
    Q_INTERFACES(PluginInterface)

public:
    PluginType getType() override;
    QVariant translateJausMsg(QVariant msg) const override;
    QVariant translateRosMsg(QVariant msg) const override;
    std::string getMaintainer() const override;
};

#endif // WRENCH_EFFORT_PLUGIN_H