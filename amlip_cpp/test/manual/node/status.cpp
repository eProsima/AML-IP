// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file directwriter.cpp
 *
 */

#include <thread>

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_event/SignalEventHandler.hpp>

#include <types/AmlipIdDataType.hpp>
#include <node/StatusNode.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::ddsrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test DirectWriter execution. Creating Node...");

    {
        // Create Status Node
        eprosima::amlip::node::StatusNode status_node("TestStatusNode");

        logUser(AMLIP_MANUAL_TEST, "Node created. Processing data asynchronously...");

        // Create callback that only prints by stdout the status that arrives
        status_node.process_status_async(
            [&status_node](const eprosima::amlip::types::StatusDataType& data)
            {
                logUser(
                    AMLIP_MANUAL_TEST,
                    "Status: " << data << " received by :" << status_node.id() << ".");
            });

        logUser(AMLIP_MANUAL_TEST, "Already processing status data. Waiting SIGINT (C^)...");

        eprosima::ddsrouter::event::SignalEventHandler<SIGINT> sigint_handler;
        sigint_handler.wait_for_event();

        logUser(AMLIP_MANUAL_TEST, "SIGINT receibed. Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test DirectWriter execution.");

    return 0;
}
