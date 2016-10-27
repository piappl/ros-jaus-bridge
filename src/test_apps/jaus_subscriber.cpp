#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/mobility.h>
#include <openjaus/system/Application.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

bool running{false};

void signalHandler(int signal)
{
    running = false;
}

bool jausCallback(openjaus::mobility::SetWrenchEffort &msg)
{
    std::cout << "Received JAUS message with data [" <<
                 msg.getPropulsiveLinearEffortX_percent() << ", " <<
                 msg.getPropulsiveLinearEffortY_percent() << ", " <<
                 msg.getPropulsiveLinearEffortZ_percent() << "]" << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "JAUS address [this JAUS component name]: ";
    std::string jaus_address{};
    std::cin >> jaus_address;

    openjaus::core::Base base_component{jaus_address};
    base_component.addMessageCallback(jausCallback);
    base_component.run();

    openjaus::system::Application::setTerminalMode();

    running = true;
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    base_component.stop();

    return 0;
}