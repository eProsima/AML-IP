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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <thread>

#include <amlip_cpp/node/MainNode.hpp>
#include <amlip_cpp/node/ComputingNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace test {

types::JobSolutionDataType computing_process_routine(
        const types::JobDataType& job_data)
{
    // Get string from data
    std::string data_str(static_cast<char*>(job_data.data()), job_data.data_size());
    // Convert str to uppercase
    std::transform(data_str.begin(), data_str.end(), data_str.begin(), ::toupper);

    // Create solution
    char* solution_data = static_cast<char*>(malloc(job_data.data_size()));
    memcpy(solution_data, data_str.c_str(), job_data.data_size());

    // Create solution data type and give ownership
    types::JobSolutionDataType solution(solution_data, job_data.data_size(), true);

    return solution;
}

uint32_t NUMBER_OF_NODES_IN_TEST = 3;

} /* namespace test */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

using namespace eprosima::amlip;

/**
 * Check that having 1 main and 1 computing node, they can communicate and resolve a job.
 * Job used is string to uppercase.
 *
 * STEPS:
 * - Create one Main Node
 * - Create one Computing Node
 * - Execute process msg in Computing in new thread
 * - Send Job (as string) from main to Computing
 * - Check solution response
 */
TEST(MainComputingNodeTest, one_main_one_computing)
{
    // Create one Main Node
    node::MainNode main_node("MainTestNode");
    types::AmlipIdDataType main_id = main_node.id();

    // Create one Computing Node
    node::ComputingNode computing_node("CompTestNode");

    // Send a new thread for computing to process message
    std::thread computing_thread(
        [&computing_node, &main_id]()
        {
            eprosima::amlip::types::AmlipIdDataType client_id;
            computing_node.process_job(node::test::computing_process_routine, client_id);

            // Check the reference comes from this test main
            EXPECT_EQ(client_id, main_id);
        });

    // Create job data and process request from main
    std::string data_sent_str("test_data");
    types::JobDataType job_data(static_cast<void*>(const_cast<char*>(data_sent_str.c_str())), data_sent_str.size());
    types::JobSolutionDataType solution = main_node.request_job_solution(job_data);

    // Wait for computing to process job
    computing_thread.join();

    // Check solution is upper case of data sent
    std::transform(data_sent_str.begin(), data_sent_str.end(), data_sent_str.begin(), ::toupper);

    std::string solution_str(static_cast<char*>(solution.data()), solution.data_size());

    ASSERT_EQ(solution_str, data_sent_str);
}

/**
 * Check that having N main and 1 computing node, they can communicate and resolve jobs.
 * Job used is string to uppercase.
 * Mains are in different threads to check syncronization while speaking with computing.
 *
 * STEPS:
 * - Create N Main Node
 * - Create one Computing Node
 * - Execute process msg in Computing in new thread
 * - Send Job (as string) from every main to Computing in new threads each
 * - Check solution response
 */
TEST(MainComputingNodeTest, n_main_one_computing)
{
    // Create N Main Node
    std::vector<std::shared_ptr<node::MainNode>> main_nodes;

    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        std::shared_ptr<node::MainNode> new_main_node(new node::MainNode(std::to_string(i) + "MainTestNode"));
        main_nodes.push_back(new_main_node);
    }

    // Create one Computing Node
    node::ComputingNode computing_node("CompTestNode");

    // Send a new thread for computing to process N messages
    std::thread computing_thread(
        [&computing_node]()
        {
            for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
            {
                computing_node.process_job(node::test::computing_process_routine);
            }
        });

    std::vector<std::thread> main_threads;
    // Create job data and process request from mains in new threads
    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        std::shared_ptr<node::MainNode> main_node_to_process = main_nodes[i];
        main_threads.push_back(std::thread(
                    [main_node_to_process, i]()
                    {
                        std::string data_sent_str("test_data" + std::to_string(i));
                        types::JobDataType job_data(
                            static_cast<void*>(const_cast<char*>(data_sent_str.c_str())), data_sent_str.size());
                        types::JobSolutionDataType solution = main_node_to_process->request_job_solution(job_data);

                        // Check solution is upper case of data sent
                        std::transform(data_sent_str.begin(), data_sent_str.end(), data_sent_str.begin(), ::toupper);

                        std::string solution_str(static_cast<char*>(solution.data()), solution.data_size());

                        ASSERT_EQ(solution_str, data_sent_str);
                    }));
    }


    // Wait for computing to process job
    computing_thread.join();
    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        main_threads[i].join();
    }
}

/**
 * Check that having 1 main and N computing node, they can communicate and resolve jobs.
 * Job used is string to uppercase.
 *
 * STEPS:
 * - Create 1 Main Node
 * - Create N Computing Nodes
 * - Execute process msg in every Computing in new thread
 * - Send N Jobs (as string) from main to Computing, so each gets one
 * (it is auto syncrhonized, do not say which one will answer)
 * - Check solution responses
 */
TEST(MainComputingNodeTest, one_main_n_computing)
{
    // Create one Main Node
    node::MainNode main_node("MainTestNode");
    types::AmlipIdDataType main_id = main_node.id();

    // Create N Computing Node
    std::vector<std::shared_ptr<node::ComputingNode>> computing_nodes;
    std::vector<std::thread> computing_threads;

    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        std::shared_ptr<node::ComputingNode> new_computing_node(
            new node::ComputingNode(std::to_string(i) + "ComputingTestNode"));
        computing_nodes.push_back(new_computing_node);

        // Execute thread for each computing node
        computing_threads.push_back(
            std::thread( [new_computing_node, main_id]()
            {
                eprosima::amlip::types::AmlipIdDataType client_id;
                new_computing_node->process_job(node::test::computing_process_routine, client_id);

                // Check the reference comes from this test main
                EXPECT_EQ(client_id, main_id);
            }));
    }

    // Create job data and process request from main N times
    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        std::string data_sent_str("test_data" + std::to_string(i));
        types::JobDataType job_data(
            static_cast<void*>(const_cast<char*>(data_sent_str.c_str())), data_sent_str.size());

        types::JobSolutionDataType solution = main_node.request_job_solution(job_data);

        // Check solution is upper case of data sent
        std::transform(data_sent_str.begin(), data_sent_str.end(), data_sent_str.begin(), ::toupper);

        std::string solution_str(static_cast<char*>(solution.data()), solution.data_size());

        ASSERT_EQ(solution_str, data_sent_str);
    }

    // Wait for computing to finish
    for (uint32_t i = 0; i < node::test::NUMBER_OF_NODES_IN_TEST; ++i)
    {
        computing_threads[i].join();
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
