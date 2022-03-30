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
 * @file main_generic.cpp
 */

#include <iostream>

#include <amlip_node/node/GenericAmlipNode.hpp>
#include <amlip_node/types/GenericType.hpp>
#include <ddsrouter_event/SignalEventHandler.hpp>

using namespace eprosima;
using namespace eprosima::amlip;
using namespace eprosima::ddsrouter;

int main(
        int argc,
        char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Argument publisher|subscriber required." << std::endl;
        return -1;
    }

    std::cout << "Starting amlip_aml_cpp_generic execution." << std::endl;

    {
        node::GenericAmlipNode node;

        if (!strcmp(argv[1], "publisher"))
        {
            // ddsrouter::event::SignalEventHandler<ddsrouter::event::SIGNAL_SIGINT> signal_handler([&](int /* signal_number */ )
            // {
            //     node.stop();
            // });

            // Wait for endpoints to match
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // if (node.wait_writer_matched())
            // {
                std::string str("HelloWorld");
                types::GenericType data_to_send(static_cast<void*>((char*)str.c_str()), str.size());
                node.publish(data_to_send);
                std::cout << "Sending data: " << str << std::endl;
            // }
        }
        else if (!strcmp(argv[1], "subscriber"))
        {
            ddsrouter::event::SignalEventHandler<ddsrouter::event::SIGNAL_SIGINT> signal_handler([&](int /* signal_number */ )
            {
                node.stop();
            });

            std::cout << "Node running, press ^C to exit." << std::endl;

            std::shared_ptr<types::GenericType> received_data = node.receive();

            if (received_data->data() != nullptr)
            {
                std::cout << "Received data: " << (const char*)received_data->data() << std::endl;
            }
        }
        else
        {
            std::cerr << "Argument publisher|subscriber required." << std::endl;
            return -1;
        }
    }

    std::cout << "Stopping execution of amlip_aml_cpp_generic gently." << std::endl;

    return 0;
}
