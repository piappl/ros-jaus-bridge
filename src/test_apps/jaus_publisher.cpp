#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/core/Triggers/Resume.h>
#include <openjaus/mobility.h>
#include <openjaus/system/Application.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <signal.h>

bool running{false};

void signalHandler(int signal)
{
    running = false;
}

int main(int argc, char **argv)
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    std::default_random_engine engine{};
    std::uniform_int_distribution<> distribution{0, 100};
    auto randomize = std::bind(distribution, engine);

    std::cout << "JAUS address [where messages should be sent?]: ";
    std::string jaus_address{};
    std::cin >> jaus_address;

    openjaus::core::Base base_component{"jaus_publisher"};
    base_component.run();

    openjaus::system::Application::setTerminalMode();

    running = true;
    while (running)
    {
        openjaus::mobility::SetWrenchEffort *msg =
                new openjaus::mobility::SetWrenchEffort{};
        msg->enablePropulsiveLinearEffortX();
        msg->enablePropulsiveLinearEffortY();
        msg->enablePropulsiveLinearEffortZ();

        msg->setPropulsiveLinearEffortX_percent(randomize());
        msg->setPropulsiveLinearEffortY_percent(randomize());
        msg->setPropulsiveLinearEffortZ_percent(randomize());

        std::vector<openjaus::transport::Address> address;
        address = base_component.getSystemTree()->
                lookupComponent(jaus_address.c_str());

        if (address.size() != 0)
        {
            base_component.requestControl(address[0]);

            openjaus::core::Resume *resume = new openjaus::core::Resume{};
            resume->setDestination(address[0]);
            base_component.sendMessage(resume);

            msg->setDestination(address[0]);
            base_component.sendMessage(msg);

            std::cout << "JAUS message was sent to address [" << jaus_address <<
                         "]." << std::endl;
        }
        else
            std::cout << "JAUS address [" << jaus_address << "] not found." <<
                         std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    base_component.stop();

    return 0;
}