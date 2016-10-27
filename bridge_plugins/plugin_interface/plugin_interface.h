#ifndef RJ_BRIDGE_PLUGIN_INTERFACE_H
#define RJ_BRIDGE_PLUGIN_INTERFACE_H

#include <QtPlugin>
#include <QVariant>

#include <string>
#include <typeinfo>

struct PluginType
{
    std::string jaus_msg_type;
    std::string ros_msg_type;
};

class PluginInterface
{
public:
    virtual PluginType getType() = 0;
    virtual QVariant translateRosMsg(QVariant msg) const = 0;
    virtual QVariant translateJausMsg(QVariant msg) const = 0;
    virtual std::string getMaintainer() const = 0;

    template<typename RosMsg, typename JausMsg>
    PluginType translateType()
    {
        PluginType type;
        type.jaus_msg_type = typeid(JausMsg).name();
        type.ros_msg_type = typeid(RosMsg).name();
        return type;
    }
};

Q_DECLARE_INTERFACE(PluginInterface, "rj_bridge.PluginInterface")

#endif // RJ_BRIDGE_PLUGIN_INTERFACE_H