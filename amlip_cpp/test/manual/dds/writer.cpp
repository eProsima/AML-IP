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
 * @file writer.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Writer execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Writer...");

        // Create Writer
        std::shared_ptr<eprosima::amlip::dds::Writer<eprosima::amlip::types::AmlipIdDataType>> writer =
                participant.create_writer<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Writer. Waiting match...");

        // Wait for discover reader
        writer->wait_match();
        // Wait a bit to let the reader do the match
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        logUser(AMLIPCPP_MANUAL_TEST, "Writer has matched. Sending data...");

        // Send data
        eprosima::amlip::types::AmlipIdDataType data("TESTDATA");
        writer->publish(data);

        logUser(AMLIPCPP_MANUAL_TEST, "Writer has sent message: " << data << ". Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Writer execution.");

    // Needed for Windows
    eprosima::utils::Log::Flush();

    return 0;
}
