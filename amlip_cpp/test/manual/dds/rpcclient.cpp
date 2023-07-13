// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file rpcclient.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>
#include <dds/Participant.hpp>

#include <dds/network_utils/model_manager.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test RPC Client execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::types::AmlipIdDataType id({"RPC-CLIENT"}, {66, 11, 77, 44});
        eprosima::amlip::dds::Participant participant(id);

        logUser(AMLIPCPP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Client...");

        // Create RPC Client
        std::shared_ptr<
            eprosima::amlip::dds::RPCClient<eprosima::amlip::types::AmlipIdDataType,
            eprosima::amlip::types::AmlipIdDataType>> client =
                participant.create_rpc_client<eprosima::amlip::types::AmlipIdDataType,
                        eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        // Data to request
        eprosima::amlip::types::AmlipIdDataType data("hello world");
        // Id server to request
        eprosima::amlip::types::AmlipIdDataType id_server({"RPC-SERVER"}, {66, 11, 77, 44});

        logUser(AMLIPCPP_MANUAL_TEST,
                "Created RPCClient. Sending request model: " << data << " to Server with ID: " << id_server);

        // Send request to server
        eprosima::amlip::types::TaskId task_id = client->send_request(data, id_server);

        // Wait reply from server
        eprosima::amlip::types::AmlipIdDataType reply =
                client->get_reply(task_id);

        logUser(AMLIPCPP_MANUAL_TEST, "Client has received reply: " << reply << " . Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test RPC Client execution.");

    return 0;
}
