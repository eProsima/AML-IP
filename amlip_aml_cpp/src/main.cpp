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

#include <ddsrouter_event/SignalHandler.hpp>

#include <amlip_dds/Reader.hpp>
#include <amlip_node/ReaderNode.hpp>

using namespace eprosima;

int main(
        int,
        char**)
{
    ddsrouter::event::SignalHandler<ddsrouter::event::SIGNAL_SIGINT> signal_handler;

    std::cout << "Starting amlip_aml_cpp execution." << std::endl;

    {
        amlip::node::ReaderNode reader("HelloWorld");
        reader.start();

        std::cout << "Node running, press ^C to exit." << std::endl;

        signal_handler.wait_for_event();
    }

    std::cout << "Stopping execution of amlip_aml_cpp gently." << std::endl;

    return 0;
}
