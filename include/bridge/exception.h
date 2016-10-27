#ifndef RJ_BRIDGE_EXCEPTION_H
#define RJ_BRIDGE_EXCEPTION_H

#include <string>

namespace rj_bridge
{
    class Exception
    {
    public:
        Exception(std::string object, std::string message)
        {
            if (!object.empty())
                object_ = object;

            if (!message.empty())
                message_ = message;
        }

        std::string getMessage() const
        {
            return message_;
        }

        std::string getObject() const
        {
            return object_;
        }

        std::string what() const
        {
            return std::string{"Object [" + object_ + "] threw exception [" +
                               message_ + "]"};
        }

        std::string message_{"EMPTY MESSAGE"};
        std::string object_{"UNKNOWN"};
    };
} // namespace rj_bridge

#endif // RJ_BRIDGE_EXCEPTION_H