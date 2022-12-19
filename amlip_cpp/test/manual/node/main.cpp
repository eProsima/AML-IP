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

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/MainNode.hpp>

std::string random_string(size_t length)
{
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(length,0);
    std::generate_n( str.begin(), length, randchar );
    return str;
}

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
    eprosima::utils::Log::SetCategoryFilter(std::regex("(AMLIPCPP)"));

    logUser(AAMLIPCPP_MLIPCPP_MANUAL_TEST, "Starting Manual Test Main Node execution. Creating Node...");

    {
        // Create Main Node
        eprosima::amlip::node::MainNode main_node("CppMainNode_Manual");

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << main_node << ". Creating job...");

        int* int_ptr = new int(4);
        delete int_ptr;

        // Create job data
        std::string data_str = "<Job Data In String>";
        std::string data_str_2("<Job Data In String>");
        std::string data_str_3 = std::move(data_str_2);
        std::string data_str_4(data_str_3);
        data_str_4.clear();
        // std::string data_str = random_string(1000000000);
        std::cout << "Sending string of size: " << data_str.size() << std::endl;
        // The cast to char* is needed to avoid const in ptr
        eprosima::amlip::types::JobDataType job_data(static_cast<void*>(
                    const_cast<char*>(data_str.c_str())),
                data_str.size() + 1);

        logUser(AMLIPCPP_MANUAL_TEST, "Job data created with string: " << data_str.substr(0, 10) << ". Sending request...");

        // Send job request
        eprosima::amlip::types::AmlipIdDataType server_id;
        eprosima::amlip::types::JobSolutionDataType solution = main_node.request_job_solution(job_data, server_id);

        logUser(AMLIPCPP_MANUAL_TEST, "Solution received from server: " << server_id << ". Deserializing to string...");

        // Convert solution to string
        std::string solution_str(static_cast<char*>(solution.data()), solution.data_size());

        logUser(AMLIPCPP_MANUAL_TEST, "Solution deserialized is: " << solution_str.substr(0, 10) << ". Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Main Node execution.");

    return 0;
}
