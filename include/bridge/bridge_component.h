#ifndef RJ_BRIDGE_BRIDGE_COMPONENT_H
#define RJ_BRIDGE_BRIDGE_COMPONENT_H

#include <memory>
#include <string>

#include "jaus_component.h"
#include "plugin_interface.h"
#include "ros_node.h"

namespace rj_bridge
{
    enum class BridgeType
    {
        ROS2JAUS,
        JAUS2ROS,
    };

    class BridgeComponent
    {
    public:
        BridgeComponent(RosNodePtr ros_node, PluginInterface *plugin)
        {
            ros_node_ = ros_node;
            plugin_ = plugin;
        }

        void stop()
        {
            jaus_component_->stop();
        }

        template<typename RosMsg, typename JausMsg>
        void create(BridgeType type, std::string ros_topic,
                    std::string jaus_address, int ros_queue_size);

    protected:
        template<typename RosMsg, typename JausMsg>
        bool jausCallback(const JausMsg &msg);

        template<typename RosMsg, typename JausMsg>
        void rosCallback(const RosMsg &msg);

        BridgeType type_;
        JausComponentPtr jaus_component_;
        PluginInterface *plugin_;
        RosNodePtr ros_node_;
        std::shared_ptr<ros::Publisher> publisher_;
        std::shared_ptr<ros::Subscriber> subscriber_;
        std::string jaus_address_;
        std::string ros_topic_;
    };

    using BridgeComponentPtr = std::shared_ptr<BridgeComponent>;
} // namespace rj_bridge

template<typename RosMsg, typename JausMsg>
void rj_bridge::BridgeComponent::create(BridgeType type, std::string ros_topic,
                                        std::string jaus_address,
                                        int ros_queue_size)
{
    type_ = type;
    jaus_address_ = jaus_address;
    ros_topic_ = ros_topic;

    switch (type_)
    {
    case BridgeType::JAUS2ROS:
        publisher_.reset(new ros::Publisher{});
        *publisher_ = ros_node_->advertise<RosMsg>(ros_topic, ros_queue_size);

        jaus_component_.reset(new JausComponent{jaus_address_});
        jaus_component_->addCallback(&BridgeComponent::jausCallback<RosMsg,
                                     JausMsg>, this);
        jaus_component_->start();
        break;
    case BridgeType::ROS2JAUS:
        subscriber_.reset(new ros::Subscriber{});
        *subscriber_ = ros_node_->subscribe(ros_topic, ros_queue_size,
                                            &BridgeComponent::rosCallback<RosMsg,
                                            JausMsg>, this);

        std::string jaus_bridge_name = ros_topic + "_bridge_to_jaus";
        jaus_component_.reset(new JausComponent{jaus_bridge_name});
        jaus_component_->start();
        break;
    }
}

template<typename RosMsg, typename JausMsg>
bool rj_bridge::BridgeComponent::jausCallback(const JausMsg &msg)
{
    ROS_INFO("Received JAUS message on address [%s]. Message will be sent "
             "to ROS topic [%s].", jaus_address_.c_str(), ros_topic_.c_str());

    QVariant variant;
    variant.setValue(msg);
    auto translated_msg = plugin_->translateJausMsg(variant);
    RosMsg ros_msg = translated_msg.value<RosMsg>();

    if (ros_node_->isMasterWorking())
    {
        publisher_->publish(ros_msg);
        ROS_INFO("JAUS message from address [%s] was sent to ROS topic [%s].",
                 jaus_address_.c_str(), ros_topic_.c_str());
    }
    else
        ROS_WARN("ROS Master is not running. Message will not be sent.");

    return true;
}

template<typename RosMsg, typename JausMsg>
void rj_bridge::BridgeComponent::rosCallback(const RosMsg &msg)
{
    ROS_INFO("Received ROS message on topic [%s]. Message will be sent to "
             "JAUS address [%s].", ros_topic_.c_str(), jaus_address_.c_str());

    RosMsg ros_msg{msg};

    QVariant variant;
    variant.setValue(ros_msg);
    auto translated_msg = plugin_->translateRosMsg(variant);
    JausMsg jaus_msg{translated_msg.value<JausMsg>()};
    JausMsg *msg_ptr{new JausMsg{jaus_msg}};

    auto address = jaus_component_->findComponent(jaus_address_);

    if (address.size() != 0)
    {
        jaus_component_->requestControl(address);
        msg_ptr->setDestination(address[0]);
        jaus_component_->sendMessage(msg_ptr);

        ROS_INFO("ROS message from topic [%s] was sent to JAUS address [%s].",
                 ros_topic_.c_str(), jaus_address_.c_str());
    }
    else
        ROS_WARN("JAUS address [%s] not found. Message rejected.",
                 jaus_address_.c_str());
}

#endif // RJ_BRIDGE_BRIDGE_COMPONENT_H