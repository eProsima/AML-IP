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
 * @file reader.cpp
 *
 */

#include <ddsrouter_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::ddsrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test Reader execution.");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIP_MANUAL_TEST, "Created Participant: " << participant << ".");

        // Create Reader
        std::shared_ptr<eprosima::amlip::dds::Reader<eprosima::amlip::types::AmlipIdDataType>> reader =
            participant.create_reader<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIP_MANUAL_TEST, "Created Reader.");

        // Wait for data
        reader->wait_data_available();

        logUser(AMLIP_MANUAL_TEST, "Data received.");

        // Read data
        eprosima::amlip::types::AmlipIdDataType received_data = reader->read();

        logUser(AMLIP_MANUAL_TEST, "Read message: " << received_data << ".");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Reader execution.");

    return 0;
}