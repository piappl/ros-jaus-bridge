#ifndef RJ_BRIDGE_ROS_NODE_H
#define RJ_BRIDGE_ROS_NODE_H

#include <ros/master.h>
#include <ros/ros.h>

#include <functional>
#include <memory>
#include <string>

namespace rj_bridge
{
    class RosNode
    {
    public:
        RosNode(int argc, char **argv, std::string node_name)
        {
            ros::init(argc, argv, node_name);
            node_handle_.reset(new ros::NodeHandle{});

            node_name_ = node_name;
        }

        bool isMasterWorking() const
        {
            return ros::master::check();
        }

        std::string getNodeName() const
        {
            return node_name_;
        }

        void start(double rate)
        {
            ros::Rate loop_rate{rate};

            while (ros::ok())
            {
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        template<typename MessageType>
        ros::Publisher advertise(std::string topic, int queue_size)
        {
            return node_handle_->advertise<MessageType>(topic, queue_size);
        }

        template<typename CallbackClass, typename MessageType>
        ros::Subscriber subscribe(std::string topic, int queue_size,
                                  void (CallbackClass::*callback)(const MessageType &),
                                  CallbackClass *object)
        {
            return node_handle_->subscribe(topic, queue_size, callback, object);
        }

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_;
        std::string node_name_{"unnamed_node"};
    };

    using RosNodePtr = std::shared_ptr<RosNode>;
} // namespace rj_bridge

#endif // RJ_BRIDGE_ROS_NODE_H