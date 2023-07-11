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
 * @file rpcserver.cpp
 *
 */

#include <algorithm>
#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>
#include <dds/Participant.hpp>

#include <dds/network_utils/model_manager.hpp>

class CustomRequestReplier : public eprosima::amlip::dds::RequestReplier<eprosima::amlip::types::AmlipIdDataType,
            eprosima::amlip::types::AmlipIdDataType>
{
public:

    CustomRequestReplier()
    {
    }

    virtual eprosima::amlip::types::AmlipIdDataType process_request (
            eprosima::amlip::types::AmlipIdDataType request) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Processing data: " << request << " . Processing data...");

        // Create new solution from data
        auto name = request.base64_name();
        std::transform(name.begin(), name.end(), name.begin(), ::toupper);

        eprosima::amlip::types::AmlipIdDataType solution(
            name, request.id());

        logUser(AMLIPCPP_MANUAL_TEST, "Processed model: " << solution << " . Returning model...");

        return solution;
    }

};

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test RPC Server execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::types::AmlipIdDataType id({"RPC-SERVER"}, {66, 11, 77, 44});
        eprosima::amlip::dds::Participant participant(id);

        logUser(AMLIPCPP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Server...");

        // Create RPC Server
        std::shared_ptr<
            eprosima::amlip::dds::RPCServer<eprosima::amlip::types::AmlipIdDataType,
            eprosima::amlip::types::AmlipIdDataType>> server =
                participant.create_rpc_server<eprosima::amlip::types::AmlipIdDataType,
                        eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Server with ID: " << participant << ". Waiting request to reply...");

        // Create listener
        std::shared_ptr<CustomRequestReplier> replier =
                std::make_shared<CustomRequestReplier>();

        // Wait request from client
        eprosima::amlip::types::RpcRequestDataType<eprosima::amlip::types::AmlipIdDataType> rpc_request =
                server->get_request(eprosima::amlip::dds::utils::WAIT_MS);

        // Calculate solution from data received from client
        eprosima::amlip::types::AmlipIdDataType rpc_solution =
                replier->process_request(rpc_request.data());

        // Create reply
        eprosima::amlip::types::RpcReplyDataType<eprosima::amlip::types::AmlipIdDataType> rpc_reply(
            rpc_request.client_id(),
            rpc_request.task_id(),
            participant.id(),
            std::move(rpc_solution));

        // Send reply to client
        server->send_reply(rpc_reply, eprosima::amlip::dds::utils::WAIT_MS);

        logUser(AMLIPCPP_MANUAL_TEST, "Server has processed model.");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test RPC Server execution.");

    return 0;
}
