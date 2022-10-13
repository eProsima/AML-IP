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
 * @file computing.cpp
 *
 */

#include <thread>

#include <ddsrouter_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <node/ComputingNode.hpp>

/*
 * The job in this example will be a string serialized to bytes.
 */
int main(
        int argc,
        char** argv)
{
    // initialize the random number generator
    srand (time(NULL));

    // Activate log
    eprosima::ddsrouter::utils::Log::SetVerbosity(eprosima::ddsrouter::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test Computing Node execution. Creating Node...");

    {
        // Create Computing Node
        eprosima::amlip::node::ComputingNode computing_node("TestComputingNode");

        logUser(AMLIP_MANUAL_TEST, "Node created: " << computing_node << ". Answering job request...");

        // Answer job request
        eprosima::amlip::types::MsReferenceDataType reference = computing_node.process_job(
            [](const eprosima::amlip::types::JobDataType& data)
            {
                // Convert data to string
                std::string data_str(static_cast<char*>(data.data()), data.data_size());

                logUser(
                    AMLIP_MANUAL_TEST,
                    "Job data: " << data_str << " received.");

                // Mangling string converting it to upper case
                std::transform(data_str.begin(), data_str.end(),data_str.begin(), ::toupper);

                logUser(
                    AMLIP_MANUAL_TEST,
                    "Job solution: " << data_str << ".");

                // Create and allocate ptr that will be then released by the Generic Data Type
                char* ptr_data = static_cast<char*>(malloc(data_str.size()));
                std::memcpy(ptr_data, data_str.c_str(), data_str.size());

                // Sending result as solution
                return eprosima::amlip::types::SolutionDataType(
                    static_cast<void*>(ptr_data), data_str.size(), true);
            });

        // Let a bit of time for the solution to be sent
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        logUser(AMLIP_MANUAL_TEST, "Answered job task: " << reference << ". Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Computing Node execution.");

    return 0;
}
