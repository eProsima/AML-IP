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
 * @file several_computing.cpp
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
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Several Computing Nodes execution. Creating Node...");

    {
        // Create Computing Nodes
        eprosima::amlip::node::ComputingNode computing_node_1("TestComputingNode_1");
        eprosima::amlip::node::ComputingNode computing_node_2("TestComputingNode_2");

        logUser(AMLIPCPP_MANUAL_TEST, "Node_1 created: " << computing_node_1 << ". Answering job request...");
        logUser(AMLIPCPP_MANUAL_TEST, "Node_2 created: " << computing_node_2 << ". Answering job request...");

        // Answer job_1 request
        eprosima::amlip::types::MsReferenceDataType reference_1 = computing_node_1.process_job(
            [](const eprosima::amlip::types::JobDataType& data_1)
            {
                // Convert data to string
                std::string data_str_1(static_cast<char*>(data_1.data()), data_1.data_size());

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job_1 data: " << data_str_1 << " received.");

                // Mangling string converting it to upper case
                std::transform(data_str_1.begin(), data_str_1.end(), data_str_1.begin(), ::toupper);

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job_1 solution: " << data_str_1 << ".");

                // Create and allocate ptr that will be then released by the Generic Data Type
                char* ptr_data = static_cast<char*>(malloc(data_str_1.size()));
                std::memcpy(ptr_data, data_str_1.c_str(), data_str_1.size());

                // Sending result as solution
                return eprosima::amlip::types::SolutionDataType(
                    static_cast<void*>(ptr_data), data_str_1.size(), true);
            });

        // Answer job_2 request
        eprosima::amlip::types::MsReferenceDataType reference_2 = computing_node_2.process_job(
            [](const eprosima::amlip::types::JobDataType& data_2)
            {
                // Convert data to string
                std::string data_str_2(static_cast<char*>(data_2.data()), data_2.data_size());

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job_2 data: " << data_str_2 << " received.");

                // Mangling string converting it to upper case
                std::transform(data_str_2.begin(), data_str_2.end(), data_str_2.begin(), ::toupper);

                logUser(
                    AMLIPCPP_MANUAL_TEST,
                    "Job_2 solution: " << data_str_2 << ".");

                // Create and allocate ptr that will be then released by the Generic Data Type
                char* ptr_data = static_cast<char*>(malloc(data_str_2.size()));
                std::memcpy(ptr_data, data_str_2.c_str(), data_str_2.size());

                // Sending result as solution
                return eprosima::amlip::types::SolutionDataType(
                    static_cast<void*>(ptr_data), data_str_2.size(), true);
            });

        // Let a bit of time for the solution to be sent
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        logUser(AMLIPCPP_MANUAL_TEST, "Answered job_1 task: " << reference_1 << ". Destroying entities...");
        logUser(AMLIPCPP_MANUAL_TEST, "Answered job_2 task: " << reference_2 << ". Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Computing Node execution.");

    return 0;
}