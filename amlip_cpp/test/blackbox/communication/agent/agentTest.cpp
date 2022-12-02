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

#include <atomic>
#include <thread>

#include <cpp_utils/utils.hpp>
#include <cpp_utils/Log.hpp>

#include <ddsrouter_core/types/address/Address.hpp>

#include <amlip_cpp/node/MainNode.hpp>
#include <amlip_cpp/node/ComputingNode.hpp>
#include <amlip_cpp/node/agent/ClientNode.hpp>
#include <amlip_cpp/node/agent/ServerNode.hpp>
#include <amlip_cpp/node/agent/TurnNode.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

namespace test {

constexpr const uint32_t N_JOBS_TO_SEND = 40;
constexpr const uint32_t N_CLIENTS = 3;

eprosima::amlip::types::JobSolutionDataType process_routine(const eprosima::amlip::types::JobDataType& job)
{
    std::string result = job.to_string();
    eprosima::utils::to_lowercase(result);
    return eprosima::amlip::types::JobSolutionDataType(result);
}

} /* namespace test */

using namespace eprosima::amlip;

/**
 *     Domain [10]                       Domain [11]
 * Main  <->  AgentClient  <->  AgentServer  <->  ComputingNode
 */
TEST(agentTest, client_server)
{
    // "Global" reference to assure that messages are not being sent before agent nodes
    std::atomic<unsigned int> n_jobs_solved(0u);

    // Create Main in its own Thread
    auto main_node_routine = [&n_jobs_solved](){
        node::MainNode main_node("TestMainNode", 10);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND; i++)
        {
            std::string job_str = std::string("TEST_SEND") + std::to_string(i);
            std::string solution_expected = std::string("test_send") + std::to_string(i);
            types::JobSolutionDataType solution =  main_node.request_job_solution(types::JobDataType(job_str));
            ASSERT_EQ(solution.to_string(), solution_expected);
            n_jobs_solved++;
        }
    };
    std::thread main_thread(main_node_routine);

    // Create Computing in its own Thread
    auto computing_node_routine = [](){
        node::ComputingNode computing_node("TestComputingNode", 11);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND; i++)
        {
            computing_node.process_job(test::process_routine);
        }
    };
    std::thread computing_thread(computing_node_routine);

    auto address = eprosima::ddsrouter::core::types::Address(
        16161, 16161, "localhost", eprosima::ddsrouter::core::types::TransportProtocol::udp);

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent client node in domain 10
    node::agent::ClientNode client_node(
        "TestAgentClientNode",
        {address},
        10u);

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent Server node in domain 11
    node::agent::ServerNode server_node(
        "TestAgentServerNode",
        {address},
        11u);

    // Wait for all messages to be sent and received
    main_thread.join();
    computing_thread.join();

    // All messages sent and received
    ASSERT_EQ(n_jobs_solved, test::N_JOBS_TO_SEND);

    // Let all entities destroy themselves correctly
}

/**
 *     Domain [10]                                      Domain [11]
 * Main  <->  AgentClient  <->  AgentTurn  <->  AgentClient  <->  ComputingNode
 */
TEST(agentTest, client_turn_client)
{
    // "Global" reference to assure that messages are being sent
    std::atomic<unsigned int> n_jobs_solved(0u);

    // Create Main in its own Thread
    auto main_node_routine = [&n_jobs_solved](){
        node::MainNode main_node("TestMainNode", 10);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND; i++)
        {
            std::string job_str = std::string("TEST_SEND") + std::to_string(i);
            std::string solution_expected = std::string("test_send") + std::to_string(i);
            types::JobSolutionDataType solution =  main_node.request_job_solution(types::JobDataType(job_str));
            ASSERT_EQ(solution.to_string(), solution_expected);
            n_jobs_solved++;
        }
    };
    std::thread main_thread(main_node_routine);

    // Create Computing in its own Thread
    auto computing_node_routine = [](){
        node::ComputingNode computing_node("TestComputingNode", 11);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND; i++)
        {
            computing_node.process_job(test::process_routine);
        }
    };
    std::thread computing_thread(computing_node_routine);

    auto address = eprosima::ddsrouter::core::types::Address(
        16161, 16161, "localhost", eprosima::ddsrouter::core::types::TransportProtocol::udp);

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent client node in domain 10
    node::agent::ClientNode client_node_10(
        "TestAgentClient10Node",
        {address},
        10u);

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent Client node in domain 11
    node::agent::ClientNode client_node_11(
        "TestAgentClient11Node",
        {address},
        11u);

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent Turn node
    node::agent::TurnNode turn_node("TestAgentTurn", {address});

    // Wait for all messages to be sent and received
    main_thread.join();
    computing_thread.join();
    ASSERT_EQ(n_jobs_solved, test::N_JOBS_TO_SEND);

    // Let all entities destroy themselves correctly
}

/**
 *     Domain [x > 10]                                    Domain [10]
 * Main  <->  AgentClient  <->  AgentTurn  <->  AgentClient  <->  Computing
 */
TEST(agentTest, turn_n_clients)
{
    // "Global" reference to assure that messages are being sent
    std::atomic<unsigned int> n_jobs_solved(0u);

    // Create each Main in its own Thread
    auto main_node_routine = [&n_jobs_solved](const unsigned int domain_id){
        node::MainNode main_node("TestMainNode", domain_id);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND; i++)
        {
            std::string job_str = std::string("TEST_SEND_") + std::to_string(i) + std::string("_") + std::to_string(domain_id);
            std::string solution_expected = std::string("test_send_") + std::to_string(i) + std::string("_") + std::to_string(domain_id);
            types::JobSolutionDataType solution =  main_node.request_job_solution(types::JobDataType(job_str));
            ASSERT_EQ(solution.to_string(), solution_expected);
            n_jobs_solved++;
        }
    };

    // Create Computing in its own Thread
    auto computing_node_routine = [](){
        node::ComputingNode computing_node("TestComputingNode", 10);
        for (unsigned int i=0; i<test::N_JOBS_TO_SEND*test::N_CLIENTS; i++)
        {
            computing_node.process_job(test::process_routine);
        }
    };
    std::thread computing_thread(computing_node_routine);

    // Create a vector to store each main thread and its associated agent client and 1 for computing
    std::array<std::thread, test::N_CLIENTS> main_threads;
    std::array<std::unique_ptr<node::agent::ClientNode>, test::N_CLIENTS + 1> clients_nodes;

    auto address = eprosima::ddsrouter::core::types::Address(
        16161, 16161, "localhost", eprosima::ddsrouter::core::types::TransportProtocol::udp);

    for (unsigned int i=0u; i<test::N_CLIENTS; ++i)
    {
        // Create Agent client node in domain i
        auto domain_id = i + 11u;
        main_threads[i] = std::thread(main_node_routine, domain_id);
        clients_nodes[i].reset(new node::agent::ClientNode(
            std::string(std::string("TestAgentClientNode") + std::to_string(domain_id)).c_str(),
            {address},
            domain_id));
    }
    // Client node for computing
    clients_nodes[test::N_CLIENTS].reset(new node::agent::ClientNode(
            "TestAgentClientNodeComputing",
            {address},
            10u));

    // No messages sent yet
    ASSERT_EQ(n_jobs_solved, 0u);

    // Create Agent Turn node
    node::agent::TurnNode turn_node("TestAgentTurn", {address});

    // Wait for all messages to be sent and received
    for (unsigned int i=0u; i<test::N_CLIENTS; ++i)
    {
        main_threads[i].join();
    }
    computing_thread.join();
    ASSERT_EQ(n_jobs_solved, test::N_CLIENTS * test::N_JOBS_TO_SEND);

    // Let all entities destroy themselves correctly
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);
    return RUN_ALL_TESTS();
}
