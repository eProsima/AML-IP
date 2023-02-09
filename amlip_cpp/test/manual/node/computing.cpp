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

#include <algorithm>
#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/ComputingNode.hpp>

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
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Computing Node execution. Creating Node...");

    {
        // Create Computing Node
        eprosima::amlip::node::ComputingNode computing_node("CppComputingNode_Manual");

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << computing_node << ". Answering job request...");

        // Answer job request
        eprosima::amlip::types::AmlipIdDataType client_id;
        computing_node.process_job(
            [](const eprosima::amlip::types::JobDataType& data)
            {
                // Convert data to string
                std::string data_str(static_cast<char*>(data.data()), data.data_size());

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job data: " << data_str << " received.");

                // Mangling string converting it to upper case
                std::transform(data_str.begin(), data_str.end(), data_str.begin(), ::toupper);

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job solution: " << data_str << ".");

                // Create and allocate ptr that will be then released by the Generic Data Type
                char* ptr_data = static_cast<char*>(malloc(data_str.size()));
                std::memcpy(ptr_data, data_str.c_str(), data_str.size());

                // Sending result as solution
                return eprosima::amlip::types::JobSolutionDataType(
                    static_cast<void*>(ptr_data), data_str.size(), true);
            },
            client_id);

        logUser(AMLIPCPP_MANUAL_TEST, "Answered job task from client: " << client_id << ". Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Computing Node execution.");

    return 0;
}
