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
 * @file main.cpp
 *
 */

#include <thread>

#include <ddsrouter_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <node/MainNode.hpp>

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

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test Main Node execution. Creating Node...");

    {
        // Create Main Node
        eprosima::amlip::node::MainNode main_node("TestMainNode");

        logUser(AMLIP_MANUAL_TEST, "Node created: " << main_node << ". Creating job...");

        // Create job data
        std::string data_str = "<Job Data In String>";
        // The cast to char* is needed to avoid const in ptr
        eprosima::amlip::types::JobDataType job_data(static_cast<void *>(const_cast<char*>(data_str.c_str())), data_str.size());

        logUser(AMLIP_MANUAL_TEST, "Job data created with string: " << data_str << ". Sending request...");

        // Send job request
        eprosima::amlip::types::SolutionDataType solution = main_node.request_job_solution(job_data);

        logUser(AMLIP_MANUAL_TEST, "Solution received. Deserializing to string...");

        // Convert solution to string
        std::string solution_str(static_cast<char *>(solution.data()), solution.data_size());

        logUser(AMLIP_MANUAL_TEST, "Solution deserialized is: " << solution_str << ". Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Main Node execution.");

    return 0;
}
