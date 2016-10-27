#ifndef RJ_BRIDGE_BRIDGE_CORE_H
#define RJ_BRIDGE_BRIDGE_CORE_H

#include <QtCore/QDir>
#include <QtCore/QPluginLoader>

#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

#include "bridge_component.h"
#include "exception.h"
#include "plugin_interface.h"
#include "ros_node.h"

namespace rj_bridge
{
    class BridgeCore
    {
    public:
        BridgeCore(int argc, char **argv, std::string node_name,
                   std::string plugins_path);

        template<typename JausMsg, typename RosMsg>
        void createBridgeJ2R(std::string jaus_bridge_name,
                             std::string ros_dst_topic,
                             int ros_queue_size = 100);
        template<typename RosMsg, typename JausMsg>
        void createBridgeR2J(std::string src_topic_name,
                             std::string jaus_dst_address,
                             int ros_queue_size = 100);
        void run(double rate = 10);

    protected:
        template<typename T>
        std::string getMsgType()
        {
            return typeid(T).name();
        }

        PluginInterface* findPlugin(std::string ros_msg_type,
                                    std::string jaus_msg_type);

        void loadAllPlugins(std::string path);
        void loadPlugin(QString file_name);

        RosNodePtr ros_node_;
        std::vector<BridgeComponentPtr> bridge_components_;
        std::vector<PluginInterface*> plugins_;
    };
} // namespace rj_bridge

template<typename RosMsg, typename JausMsg>
void rj_bridge::BridgeCore::createBridgeR2J(std::string ros_src_topic,
                                            std::string jaus_dst_address,
                                            int ros_queue_size)
{
    ROS_INFO("Creating ROS->Jaus bridge [ROS topic: %s -> JAUS address: %s]",
             ros_src_topic.c_str(), jaus_dst_address.c_str());

    if (jaus_dst_address.empty())
        throw Exception{"BridgeCore::createBridgeR2J()",
                        "Jaus destination address is empty."};

    if (ros_src_topic.empty())
        throw Exception{"BridgeCore::createBridgeR2J()",
                        "ROS source topic name is empty."};

    if (ros_queue_size < 0)
        throw Exception{"BridgeCore::createBridgeR2J()",
                        "ROS message queue size is invalid (< 0)."};

    PluginInterface *plugin = findPlugin(getMsgType<RosMsg>(),
                                         getMsgType<JausMsg>());

    if (plugin)
        ROS_INFO("Plugin for specified transformation found.");
    else
    {
        ROS_ERROR("Plugin for specified transformation not found. Bridge will "
                  "not be created.");
        return;
    }

    BridgeComponentPtr bridge_cmp{new BridgeComponent{ros_node_, plugin}};
    bridge_cmp->create<RosMsg, JausMsg>(BridgeType::ROS2JAUS, ros_src_topic,
                                        jaus_dst_address, ros_queue_size);
    bridge_components_.push_back(bridge_cmp);
}

template<typename JausMsg, typename RosMsg>
void rj_bridge::BridgeCore::createBridgeJ2R(std::string jaus_bridge_name,
                                            std::string ros_dst_topic,
                                            int ros_queue_size)
{
    ROS_INFO("Creating Jaus->ROS bridge [JAUS bridge address: %s -> ROS topic: %s]",
             jaus_bridge_name.c_str(), ros_dst_topic.c_str());

    if (jaus_bridge_name.empty())
        throw Exception{"BridgeCore::createBridgeJ2R()",
                        "Jaus bridge component name is empty."};

    if (ros_dst_topic.empty())
        throw Exception{"BridgeCore::createBridgeJ2R()",
                        "ROS destination topic name is empty."};

    if (ros_queue_size < 0)
        throw Exception{"BridgeCore::createBridgeJ2R()",
                        "ROS message queue size is invalid (< 0)."};

    PluginInterface *plugin = findPlugin(getMsgType<RosMsg>(),
                                         getMsgType<JausMsg>());

    if (plugin)
        ROS_INFO("Plugin for specified transformation found.");
    else
    {
        ROS_ERROR("Plugin for specified transformation not found. Bridge will "
                  "not be created.");
        return;
    }

    BridgeComponentPtr bridge_cmp{new BridgeComponent{ros_node_, plugin}};
    bridge_cmp->create<RosMsg, JausMsg>(BridgeType::JAUS2ROS, ros_dst_topic,
                                        jaus_bridge_name, ros_queue_size);
    bridge_components_.push_back(bridge_cmp);
}

#endif // RJ_BRIDGE_BRIDGE_CORE_H