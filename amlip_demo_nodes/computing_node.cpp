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
 * @file computing_node.cpp
 *
 */

#include <algorithm>
#include <iostream>
#include <random>
#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/node/ComputingNode.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

using namespace eprosima::amlip;

// Routine to process a Task and return a Solution
types::JobSolutionDataType routine_process_task(
        const types::JobDataType& job)
{
    // Get string from job
    std::string job_str = job.to_string();
    std::cout << " Received Job: <" << job_str << ">. Processing..." << std::endl;

    // Wait for a random time between 2 and 7 seconds
    auto time_to_wait = (rand() % 5) + 2;
    std::this_thread::sleep_for(std::chrono::seconds(time_to_wait));

    // Get solution by converting string to uppercase
    std::transform(job_str.begin(), job_str.end(), job_str.begin(), [](unsigned char c)
            {
                return std::toupper(c);
            });
    std::cout << " Answering Solution: <" << job_str << ">." << std::endl;
    types::JobSolutionDataType solution(job_str);

    return solution;
}

int main(
        int argc,
        char** argv)
{
    // initialize the random number generator
    srand(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count());

    // Get argument to know how many tasks to answer
    int received = 1;
    if (argc > 1)
    {
        received = std::strtol(argv[1], nullptr, 10);
    }

    {
        // Create Node
        node::ComputingNode node("AMLComputingNode");
        std::cout << "Computing Node " << node.id() << " computing " << received << " tasks." << std::endl;

        // Answer tasks until signal received
        while (received--)
        {
            // Answer job request
            node.process_job(routine_process_task);

            std::cout << "Computing Node " << node.id() << " answered task. " << received << " remaining." << std::endl;
        }

        // Closing
        std::cout << "Computing Node " << node.id() << " closing." << std::endl;
    }

    return 0;
}
