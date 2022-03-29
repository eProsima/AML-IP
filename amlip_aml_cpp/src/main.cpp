// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file main.cpp
 */

#include <iostream>

#include <amlip_node/node/StatusAmlipNode.hpp>
#include <ddsrouter_event/SignalEventHandler.hpp>

using namespace eprosima;
using namespace eprosima::amlip;

int main(
        int,
        char**)
{
    std::cout << "Starting amlip_aml_cpp execution." << std::endl;

    {
        node::StatusAmlipNode status_node(
            [](types::Status status)
            {
                std::cout << "Specific CPP Status read <" << status << "> ." << std::endl;
            });

        ddsrouter::event::SignalEventHandler<ddsrouter::event::SIGNAL_SIGINT> signal_handler([&status_node](int /* signal_number */ )
            {
                status_node.stop();
            });

        std::cout << "Node running, press ^C to exit." << std::endl;
        status_node.spin();
    }

    std::cout << "Stopping execution of amlip_aml_cpp gently." << std::endl;

    return 0;
}
