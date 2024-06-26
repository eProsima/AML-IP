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
 * @file agent_server.cpp
 *
 */

#include <algorithm>
#include <thread>

#include <cpp_utils/event/SignalEventHandler.hpp>
#include <cpp_utils/Log.hpp>

#include <ddspipe_participants/types/address/Address.hpp>

#include <amlip_cpp/node/wan/ServerNode.hpp>

int main(
        int argc,
        char** argv)
{

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Agent Server Node execution. Creating listening address...");

    {
        // Create listening address
        auto listening_address = eprosima::ddspipe::participants::types::Address(
            12121,
            12121,
            "localhost",
            eprosima::ddspipe::participants::types::TransportProtocol::udp);

        logUser(AMLIPCPP_MANUAL_TEST, "Address where listen: " << listening_address << ". Creating Node...");

        // Create Server Node
        eprosima::amlip::node::agent::ServerNode Client_node(
            "CppServerNode_Manual",
            { listening_address });

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << Client_node << ". Waiting SIGINT (C^) to close...");

        eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
        sigint_handler.wait_for_event();

        logUser(AMLIPCPP_MANUAL_TEST, "SIGINT received. Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Server Node execution.");

    return 0;
}
