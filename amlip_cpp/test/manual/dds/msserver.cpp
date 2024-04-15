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
 * @file msserver.cpp
 *
 */

#include <algorithm>
#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

// To process the data, it will be created a new id with same id_num but in uppercase
eprosima::amlip::types::AmlipIdDataType server_callback(
        const eprosima::amlip::types::AmlipIdDataType& data)
{
    logUser(AMLIPCPP_MANUAL_TEST, "Processing data: " << data << " . Processing data...");

    // Create new solution from data
    auto name = data.base64_name();
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);

    eprosima::amlip::types::AmlipIdDataType solution(
        name, data.id());

    logUser(AMLIPCPP_MANUAL_TEST, "Processed solution: " << solution << " . Returning solution...");

    return solution;
}

int main(
        int argc,
        char** argv)
{
    // Activate log
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test MultiService Server execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Server...");

        // Create Writer
        std::shared_ptr<
            eprosima::amlip::dds::MultiServiceServer<
                eprosima::amlip::types::AmlipIdDataType, eprosima::amlip::types::AmlipIdDataType>> server =
                participant.create_multiservice_server<
            eprosima::amlip::types::AmlipIdDataType, eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Server. Waiting data to process...");

        // Wait for discover reader
        eprosima::amlip::types::MsReferenceDataType reference = server->process_task_sync(server_callback);

        logUser(AMLIPCPP_MANUAL_TEST, "Server has processed task: " << reference << " . Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test MultiService Server execution.");

    return 0;
}
