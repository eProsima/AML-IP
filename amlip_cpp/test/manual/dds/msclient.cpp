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
 * @file msclient.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test MultiService Client execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Client...");

        // Create Writer
        std::shared_ptr<
            eprosima::amlip::dds::MultiServiceClient<
                eprosima::amlip::types::AmlipIdDataType, eprosima::amlip::types::AmlipIdDataType>> client =
            participant.create_multiservice_client<
                eprosima::amlip::types::AmlipIdDataType, eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        eprosima::amlip::types::AmlipIdDataType data("testdata");

        logUser(AMLIP_MANUAL_TEST, "Created Client. Sending task data: " << data << " ...");

        // Wait for discover reader
         eprosima::amlip::types::AmlipIdDataType solution = client->send_request_sync(data);

        logUser(AMLIP_MANUAL_TEST, "Client has received solution: " << solution << " . Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test MultiService Client execution.");

    return 0;
}
