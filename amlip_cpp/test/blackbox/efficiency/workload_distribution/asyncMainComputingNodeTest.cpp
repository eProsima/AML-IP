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

#include <cpp_utils/math/math_extension.hpp>
#include <cpp_utils/time/time_utils.hpp>
#include <cpp_utils/time/Timer.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/types/Atomicable.hpp>
#include <cpp_utils/wait/IntWaitHandler.hpp>

#include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>
#include <amlip_cpp/node/workload_distribution/AsyncComputingNode.hpp>

using namespace eprosima::amlip;

namespace test {

constexpr eprosima::utils::Duration_ms EXECUTION_TIME = 200;
constexpr eprosima::utils::Duration_ms METACOMMUNICATION_EXPECTED_TIME = 100;
constexpr eprosima::utils::Duration_ms MESSAGES_REPLY_EXPECTED_TIME = EXECUTION_TIME + METACOMMUNICATION_EXPECTED_TIME;
constexpr eprosima::utils::Duration_ms RESIDUAL_TIME = EXECUTION_TIME / 10;
constexpr const char* MESSAGE_INTERNAL_TEST = "Some random message that nothing matters";

using SolutionsReceivedType =
    eprosima::utils::Atomicable<
        std::map<
            types::TaskId,
            std::unique_ptr<types::JobSolutionDataType>>>;

class TestSolutionListener : public node::SolutionListener
{
public:
    void solution_received(
        std::unique_ptr<types::JobSolutionDataType> solution,
        const types::TaskId& task_id,
        const types::AmlipIdDataType& server_id) override
    {
        // Store solution
        std::lock_guard<SolutionsReceivedType> guard(solutions);
        solutions[task_id] = std::move(solution);

        // Increase in 1 the number of solutions received
        ++waiter;
    }

    SolutionsReceivedType solutions;
    eprosima::utils::event::IntWaitHandler waiter{0};
};

class TestTaskListener : public node::JobListener
{
public:
    types::JobSolutionDataType process_job (
            std::unique_ptr<types::JobDataType> job,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id) override
    {
        // Simulate time execution
        eprosima::utils::sleep_for(EXECUTION_TIME);
        return types::JobSolutionDataType(job->to_string());
    }
};

eprosima::utils::Duration_ms time_expected(unsigned int n_clients, unsigned int n_servers, unsigned int n_messages)
{
    auto total_messages = n_clients * n_messages;
    if (eprosima::utils::fast_module(total_messages, n_servers) == 0)
    {
        return eprosima::utils::fast_division(total_messages, n_servers) * MESSAGES_REPLY_EXPECTED_TIME;
    }
    else
    {
        return (eprosima::utils::fast_division(total_messages, n_servers) + 1) * MESSAGES_REPLY_EXPECTED_TIME;
    }
}

void send_messages(
    std::vector<std::unique_ptr<node::AsyncMainNode>>& main_nodes, unsigned int n_messages)
{
    std::shared_ptr<types::JobDataType> job(std::make_shared<types::JobDataType>(MESSAGE_INTERNAL_TEST));
    for (auto& main_node : main_nodes)
    {
        for (unsigned int i=0; i<n_messages; ++i)
        {
            main_node->request_job_solution(job);
        }
    }
}

void wait_solutions(
    std::vector<std::shared_ptr<test::TestSolutionListener>>& main_listeners, unsigned int n_messages)
{
    for (auto& listener : main_listeners)
    {
        listener->waiter.wait_equal(n_messages);
    }
}

eprosima::utils::Duration_ms execute_nodes(unsigned int n_clients, unsigned int n_servers, unsigned int n_messages)
{
    //////
    // COMPUTING NODES RUNNING
    std::vector<std::shared_ptr<test::TestTaskListener>> computing_listeners;
    std::vector<std::unique_ptr<node::AsyncComputingNode>> computing_nodes;
    for (unsigned int i=0; i<n_servers; i++)
    {
        computing_listeners.emplace_back(std::make_shared<test::TestTaskListener>());
        computing_nodes.emplace_back(
            std::make_unique<node::AsyncComputingNode>(
                "TestComputingNode",
                computing_listeners[i]));
        computing_nodes[i]->run();
    }

    //////
    // MAIN NODES RUNNING
    std::vector<std::shared_ptr<test::TestSolutionListener>> main_listeners;
    std::vector<std::unique_ptr<node::AsyncMainNode>> mains_nodes;
    for (unsigned int i=0; i<n_clients; i++)
    {
        main_listeners.emplace_back(std::make_shared<test::TestSolutionListener>());
        mains_nodes.emplace_back(
            std::make_unique<node::AsyncMainNode>(
                "TestMainNode",
                main_listeners[i]));
    }

    //////
    // SEND MESSAGES AND MEASURE TIME
    eprosima::utils::Timer timer;

    send_messages(mains_nodes, n_messages);
    wait_solutions(main_listeners, n_messages);

    auto elapsed_time = timer.elapsed_ms();

    return elapsed_time;
}

void execute_test(unsigned int n_clients, unsigned int n_servers, unsigned int n_messages)
{
    auto time_elapsed = execute_nodes(n_clients, n_servers, n_messages);
    ASSERT_LT(time_elapsed, time_expected(n_clients, n_servers, n_messages) + RESIDUAL_TIME);
}

} /* namespace test */

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_1_1_1)
{
    test::execute_test(1, 1, 1);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_1_1_20)
{
    test::execute_test(1, 1, 20);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_1_5_5)
{
    test::execute_test(1, 5, 5);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_1_5_23)
{
    test::execute_test(1, 5, 23);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_5_1_1)
{
    test::execute_test(5, 1, 1);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_5_1_5)
{
    test::execute_test(5, 1, 5);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_5_10_2)
{
    test::execute_test(5, 10, 2);
}

TEST(asyncMainComputingNodeTest, async_main_computing_time_test_3_5_11)
{
    test::execute_test(3, 5, 11);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
