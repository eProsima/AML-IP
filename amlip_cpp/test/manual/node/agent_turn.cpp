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
 * @file agent_turn.cpp
 *
 */

#include <algorithm>
#include <thread>

#include <cpp_utils/event/SignalEventHandler.hpp>
#include <cpp_utils/Log.hpp>
#include <ddsrouter_core/types/address/Address.hpp>

#include <amlip_cpp/node/agent/TurnNode.hpp>

int main(
        int argc,
        char** argv)
{

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Agent Turn Node execution. Creating listening address...");

    {
        // Create listening address
        auto listening_address = eprosima::ddsrouter::core::types::Address(
                    12121,
                    12121,
                    "localhost",
                    eprosima::ddsrouter::core::types::TransportProtocol::udp);

        logUser(AMLIPCPP_MANUAL_TEST, "Address where listen: " << listening_address << ". Creating Node...");

        // Create Turn Node
        eprosima::amlip::node::agent::TurnNode turn_node(
            "CppTurnNode_Manual",
            { listening_address });

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << turn_node << ". Waiting SIGINT (C^) to close...");

        eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
        sigint_handler.wait_for_event();

        logUser(AMLIPCPP_MANUAL_TEST, "SIGINT received. Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Turn Node execution.");

    return 0;
}
