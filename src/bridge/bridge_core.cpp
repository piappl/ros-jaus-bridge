#include "bridge_core.h"

using namespace rj_bridge;

BridgeCore::BridgeCore(int argc, char **argv, std::string node_name,
                       std::string plugins_path)
{
    if (node_name.empty())
        throw Exception{"BridgeCore::BridgeCore()",
                        "Node name cannot be empty."};

    if (plugins_path.empty())
        throw Exception{"BridgeCore::BridgeCore()", "Plugins path is empty."};

    ros_node_.reset(new RosNode{argc, argv, node_name});

    ROS_INFO("BridgeCore [node name = %s] created and initialized.",
             node_name.c_str());

    loadAllPlugins(plugins_path);
}

void BridgeCore::run(double rate)
{
    ROS_INFO("BridgeCore started. Listening for messages ...");

    ros_node_->start(rate);

    for (const auto &cmp: bridge_components_)
        cmp->stop();
}

void BridgeCore::loadAllPlugins(std::string path)
{
    std::string separator(64, '*');

    QDir plugins_path{QString{path.c_str()}};
    ROS_INFO("%s", separator.c_str());
    ROS_INFO("Searching for plugins in: %s ...",
             plugins_path.path().toStdString().c_str());

    foreach (QString file_name, plugins_path.entryList(QDir::Files))
        loadPlugin(plugins_path.path() + "/" + file_name);

    ROS_INFO("Plugin loader finished its job.");
    ROS_INFO("%s", separator.c_str());
}

void BridgeCore::loadPlugin(QString file_name)
{
    ROS_INFO("Loading plugin: %s ...", file_name.toStdString().c_str());

    QPluginLoader loader{file_name};
    QObject *possible_plugin{loader.instance()};

    if (possible_plugin)
    {
        PluginInterface *plugin = qobject_cast<PluginInterface*>(possible_plugin);

        ROS_INFO("Plugin loaded successfully. Plugin information:");
        ROS_INFO("--- Plugin maintainter: %s", plugin->getMaintainer().c_str());
        ROS_INFO("--- Plugin type [ROS <-> JAUS]: %s <-> %s",
                 plugin->getType().ros_msg_type.c_str(),
                 plugin->getType().jaus_msg_type.c_str());

        plugins_.push_back(plugin);
    }
    else
        ROS_WARN("Invalid plugin skipped: %s.", file_name.toStdString().c_str());
}

PluginInterface* BridgeCore::findPlugin(std::string ros_msg_type,
                                        std::string jaus_msg_type)
{
    PluginInterface *plugin_if{nullptr};

    for (const auto &plugin: plugins_)
    {
        auto type = plugin->getType();
        if (type.jaus_msg_type == jaus_msg_type &&
            type.ros_msg_type == ros_msg_type)
            plugin_if = plugin;
    }

    return plugin_if;
}