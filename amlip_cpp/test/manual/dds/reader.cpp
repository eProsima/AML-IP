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
 * @file reader.cpp
 *
 */

#include <cpp_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test Reader execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Reader...");

        // Create Reader
        std::shared_ptr<eprosima::amlip::dds::Reader<eprosima::amlip::types::AmlipIdDataType>> reader =
                participant.create_reader<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIP_MANUAL_TEST, "Created Reader. Waiting for data...");

        // Wait for data
        reader->wait_data_available();

        logUser(AMLIP_MANUAL_TEST, "Data received. Reading data...");

        // Read data
        eprosima::amlip::types::AmlipIdDataType received_data = reader->read();

        logUser(AMLIP_MANUAL_TEST, "Read message: " << received_data << ". Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Reader execution.");

    // Needed for Windows
    eprosima::utils::Log::Flush();

    return 0;
}
