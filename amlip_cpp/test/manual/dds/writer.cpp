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
 * @file writer.cpp
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

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test Writer execution.");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIP_MANUAL_TEST, "Created Participant: " << participant << ".");

        // Create Writer
        std::shared_ptr<eprosima::amlip::dds::Writer<eprosima::amlip::types::AmlipIdDataType>> writer =
                participant.create_writer<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIP_MANUAL_TEST, "Created Writer.");

        // Wait for discover reader
        writer->wait_match();

        logUser(AMLIP_MANUAL_TEST, "Writer has matched.");

        // Send data
        eprosima::amlip::types::AmlipIdDataType data("TESTDATA");
        writer->publish(data);

        logUser(AMLIP_MANUAL_TEST, "Writer has sent message: " << data << ".");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Writer execution.");

    // Needed for Windows
    eprosima::ddsrouter::utils::Log::Flush();

    return 0;
}
