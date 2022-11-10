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
 * @file several_main.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/MainNode.hpp>

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

    logUser(AAMLIPCPP_MLIPCPP_MANUAL_TEST, "Starting Manual Test Several Main Nodes execution. Creating Nodes...");

    {
        // Create Main Nodes
        eprosima::amlip::node::MainNode main_node_1("TestMainNode_1");
        eprosima::amlip::node::MainNode main_node_2("TestMainNode_2");

        logUser(AMLIPCPP_MANUAL_TEST, "Node_1 created: " << main_node_1 << ". Creating job...");
        logUser(AMLIPCPP_MANUAL_TEST, "Node_2 created: " << main_node_2 << ". Creating job...");

        // Create job datas
        std::string data_str_1 = "<Job Data 1 In String>";
        std::string data_str_2 = "<Job Data 2 In String>";

        // The cast to char* is needed to avoid const in ptr
        eprosima::amlip::types::JobDataType job_data_1(static_cast<void*>(const_cast<char*>(data_str_1.c_str())),
                data_str_1.size());
        eprosima::amlip::types::JobDataType job_data_2(static_cast<void*>(const_cast<char*>(data_str_2.c_str())),
                data_str_2.size());

        logUser(AMLIPCPP_MANUAL_TEST, "Job data 1 created with string: " << data_str_1 << ". Sending request...");
        logUser(AMLIPCPP_MANUAL_TEST, "Job data 2 created with string: " << data_str_2 << ". Sending request...");

        // Send jobs request
        eprosima::amlip::types::SolutionDataType solution_1 = main_node_1.request_job_solution(job_data_1);
        logUser(AMLIPCPP_MANUAL_TEST, "Solution 1 received. Deserializing to string...");

        eprosima::amlip::types::SolutionDataType solution_2 = main_node_2.request_job_solution(job_data_2);
        logUser(AMLIPCPP_MANUAL_TEST, "Solution 2 received. Deserializing to string...");

        // Convert solutions to string
        std::string solution_str_1(static_cast<char*>(solution_1.data()), solution_1.data_size());
        logUser(AMLIPCPP_MANUAL_TEST, "Solution 1 deserialized is: " << solution_str_1 << ". Destroying entities...");

        std::string solution_str_2(static_cast<char*>(solution_2.data()), solution_2.data_size());
        logUser(AMLIPCPP_MANUAL_TEST, "Solution 2 deserialized is: " << solution_str_2 << ". Destroying entities...");

    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Main Nodes execution.");

    return 0;
}
