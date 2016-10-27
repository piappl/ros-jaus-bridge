#ifndef RJ_BRIDGE_JAUS_COMPONENT_H
#define RJ_BRIDGE_JAUS_COMPONENT_H

#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/core/Triggers/Resume.h>

#include <memory>
#include <string>
#include <vector>

namespace rj_bridge
{
    using JausAddress = std::vector<openjaus::transport::Address>;

    class JausComponent
    {
    public:
        JausComponent(std::string name)
        {
            name_ = name;
            base_component_.reset(new openjaus::core::Base{name.c_str()});
        }

        JausAddress findComponent(std::string name)
        {
            return base_component_->getSystemTree()->
                    lookupComponent(name.c_str());
        }

        std::string getComponentName() const
        {
            return name_;
        }

        void requestControl(JausAddress address)
        {
            base_component_->requestControl(address[0]);

            openjaus::core::Resume *resume{new openjaus::core::Resume{}};
            resume->setDestination(address[0]);
            base_component_->sendMessage(resume);
        }

        void sendMessage(openjaus::model::Trigger *message)
        {
            base_component_->sendMessage(message);
        }

        void start()
        {
            base_component_->run();
        }

        void stop()
        {
            base_component_->stop();
        }

        template<typename CallbackClass, typename MessageType>
        void addCallback(bool (CallbackClass::*callback)(const MessageType &message),
                         CallbackClass *object)
        {
            base_component_->addMessageCallback(callback, object);
        }

    protected:
        std::shared_ptr<openjaus::core::Base> base_component_;
        std::string name_{"unnamed_component"};
    };

    using JausComponentPtr = std::shared_ptr<JausComponent>;
} // namespace rj_bridge

#endif // RJ_BRIDGE_JAUS_COMPONENT_H