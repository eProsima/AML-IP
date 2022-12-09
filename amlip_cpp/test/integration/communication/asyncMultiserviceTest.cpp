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

#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/types/Atomicable.hpp>
#include <cpp_utils/wait/IntWaitHandler.hpp>

#include <dds/Participant.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/GenericDataType.hpp>

namespace test {

constexpr const uint32_t N_SERVERS = 4;
constexpr const uint32_t N_CLIENTS = 8;
constexpr const uint32_t N_MESSAGES = 20;

class TestDataType : public eprosima::amlip::types::GenericDataType
{
    using GenericDataType::GenericDataType;
};

using SolutionsReceivedType =
    eprosima::utils::Atomicable<
        std::map<
            std::pair<
                eprosima::amlip::types::TaskId,
                eprosima::amlip::types::AmlipIdDataType>,
            std::pair<
                std::unique_ptr<TestDataType>,
                eprosima::amlip::types::AmlipIdDataType>>>;

class TestSolutionListener : public eprosima::amlip::dds::SolutionListener<TestDataType>
{
public:
    void solution_received(
        std::unique_ptr<TestDataType> solution,
        const eprosima::amlip::types::TaskId& task_id,
        const eprosima::amlip::types::AmlipIdDataType& client_id,
        const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        // Store solution
        std::lock_guard<SolutionsReceivedType> guard(solutions);
        solutions[{task_id, client_id}] = std::pair<std::unique_ptr<TestDataType>, eprosima::amlip::types::AmlipIdDataType>(
            std::move(solution), server_id);

        // Increase in 1 the number of solutions received
        ++waiter;
    }

    SolutionsReceivedType solutions;
    eprosima::utils::event::IntWaitHandler waiter{0};
};

class TestTaskListener : public eprosima::amlip::dds::TaskListener<TestDataType, TestDataType>
{
public:
    TestDataType process_task (
            std::unique_ptr<TestDataType> task,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& client_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        // Increase in 1 the number of solutions received
        ++waiter;

        // Return solution as string to_lowercase
        std::string result = task->to_string();
        eprosima::utils::to_lowercase(result);
        return test::TestDataType(result);
    }

    eprosima::utils::event::IntWaitHandler waiter{0};
};

} // namespace test

using namespace eprosima::amlip;
using namespace eprosima::amlip::dds;

/**
 * Communicate a Asyn MS Client and a Async MS Server from different participants
 * They use TestDataType as data type for Data and Solution and they are created as strings and processed as to_lower.
 * STEPS:
 * - Create entities and run client
 * - Send N messages from client with server no running
 * - Run server
 * - Wait for client to get all answers
 * - Send N messages again
 * - Wait for client to get all answers
 * - Check all messages received
 */
TEST(asyncMultiServiceTest, communicate_service_one_on_one)
{
    // Create participants
    Participant participant_client(std::string("ClientTestPart"));
    Participant participant_server(std::string("ServerTestPart"));
    types::AmlipIdDataType participant_client_id = participant_client.id();

    // Create a MS Client
    auto client =
            participant_client.create_async_multiservice_client<test::TestDataType, test::TestDataType>("test_topic");

    // Create a MS Server
    auto server =
            participant_server.create_async_multiservice_server<test::TestDataType, test::TestDataType>("test_topic");

    // Create listeners for each entity
    auto solution_listener = std::make_shared<test::TestSolutionListener>();
    auto task_listener = std::make_shared<test::TestTaskListener>();

    // Start entities
    client->run(solution_listener);

    // Send N messages
    for (unsigned int i=0; i<test::N_MESSAGES; i++)
    {
        // Get new task
        std::string task_str = std::string("NEW_TASK_") + std::to_string(i);

        // Send task
        client->send_request_async(std::make_shared<test::TestDataType>(task_str));
    }

    // Check that no tasks have been answered but thread arrives to here
    {
        std::lock_guard<test::SolutionsReceivedType> guard(solution_listener->solutions);
        ASSERT_EQ(solution_listener->solutions.size(), 0u);
    }

    // Run server
    server->run(task_listener);

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES);

    // Send N again and now the server is already awaken
    for (unsigned int i=0; i<test::N_MESSAGES; i++)
    {
        // Get new task
        std::string task_str = std::string("NEW_TASK_") + std::to_string(i + test::N_MESSAGES);

        // Send task
        client->send_request_async(std::make_shared<test::TestDataType>(task_str));
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * 2);

    // Check messages received
    {
        std::lock_guard<test::SolutionsReceivedType> guard(solution_listener->solutions);
        for (unsigned int i=0; i<test::N_MESSAGES * 2; i++)
        {
            std::string expected_str = std::string("new_task_") + std::to_string(i);

            std::pair<types::TaskId, types::AmlipIdDataType> p = {i, participant_client_id};
            ASSERT_EQ(solution_listener->solutions[p].first->to_string(), expected_str);
            // Check the server is this one
            ASSERT_EQ(solution_listener->solutions[p].second, participant_server.id());
        }
    }
}

/**
 * Communicate 1 Async MS Client with M Async MS Server from different participants
 * They use TestDataType as data type for Data and Solution and they are created as strings and processed as to_lower.
 * STEPS:
 * - Create entities and run client
 * - Send N messages from client with server no running
 * - Run server
 * - Wait for client to get all answers
 * - Stop server that has answered last task
 * - Send N messages again
 * - Wait for client to get all answers
 * - Check all messages received
 */
TEST(asyncMultiServiceTest, communicate_service_one_client_n_servers)
{
    // Create Client Participants and Clients
    std::shared_ptr<test::TestSolutionListener> solution_listener = std::make_shared<test::TestSolutionListener>();
    std::unique_ptr<Participant> participant_client = std::make_unique<Participant>("ClientTestPart");
    types::AmlipIdDataType participant_client_id = participant_client->id();

    auto client = participant_client->create_async_multiservice_client<test::TestDataType, test::TestDataType>(
            "test_topic");
    client->run(solution_listener);

    eprosima::amlip::types::TaskId last_task_id;

    // Send N messages
    for (unsigned int i=0; i<test::N_MESSAGES; i++)
    {
        // Get new task
        std::string task_str = std::string("NEW_TASK_") + std::to_string(i);
        // Send task
        last_task_id = client->send_request_async(std::make_shared<test::TestDataType>(task_str));
    }

    // Create Server Participants and Servers
    std::shared_ptr<test::TestTaskListener> task_listener = std::make_shared<test::TestTaskListener>();
    std::array<std::unique_ptr<Participant>, test::N_SERVERS> participant_servers;
    std::array<
        std::shared_ptr<eprosima::amlip::dds::AsyncMultiServiceServer<test::TestDataType, test::TestDataType>>,
        test::N_SERVERS
    > servers;

    // Create servers and run them sending N messages each
    for (unsigned int i=0; i<test::N_SERVERS; i++)
    {
        participant_servers[i] = std::make_unique<Participant>(std::string("ServerTestPart") + std::to_string(i));
        servers[i] = participant_servers[i]->create_async_multiservice_server<test::TestDataType, test::TestDataType>(
            "test_topic");
        servers[i]->run(task_listener);
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES);

    // Stop the server that has answered, in case the communication is not correct with the others
    auto last_task_server_id = solution_listener->solutions[{last_task_id, participant_client_id}].second;
    for (unsigned int i=0; i<test::N_SERVERS; i++)
    {
        if (participant_servers[i]->id() == last_task_server_id)
        {
            servers[i]->stop();
            break;
        }
    }

    // Send all messages again
    for (unsigned int i=0; i<test::N_MESSAGES; i++)
    {
        // Get new task
        std::string task_str = std::string("NEW_TASK_") + std::to_string(i + test::N_MESSAGES);
        // Send task
        client->send_request_async(std::make_shared<test::TestDataType>(task_str));
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * 2);

    std::lock_guard<test::SolutionsReceivedType> guard(solution_listener->solutions);
    ASSERT_EQ(solution_listener->solutions.size(), test::N_MESSAGES * 2);
    for (unsigned int i=0; i<test::N_MESSAGES * 2; i++)
    {
        std::string expected_str = std::string("new_task_") + std::to_string(i);

        std::pair<types::TaskId, types::AmlipIdDataType> p = {i, participant_client_id};
        ASSERT_EQ(solution_listener->solutions[p].first->to_string(), expected_str);
    }
}

/**
 * Communicate N Asyn MS Client with 1 Async MS Server from different participants
 * They use TestDataType as data type for Data and Solution and they are created as strings and processed as to_lower.
 * STEPS:
 * - Create entities and run client
 * - Send N messages from client with server no running
 * - Run server
 * - Wait for client to get all answers
 * - Send N messages again
 * - Wait for client to get all answers
 * - Check all messages received
 */
TEST(asyncMultiServiceTest, communicate_service_n_clients_one_server)
{
    // Create listeners for each entity
    std::shared_ptr<test::TestSolutionListener> solution_listener = std::make_shared<test::TestSolutionListener>();
    std::shared_ptr<test::TestTaskListener> task_listener = std::make_shared<test::TestTaskListener>();

    // Create Participants and Clients
    std::array<std::unique_ptr<Participant>, test::N_CLIENTS> participant_clients;
    std::array<
        std::shared_ptr<eprosima::amlip::dds::AsyncMultiServiceClient<test::TestDataType, test::TestDataType>>,
        test::N_CLIENTS
    > clients;

    // Create clients and run them sending N messages each
    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        participant_clients[i] = std::make_unique<Participant>(std::string("ClientTestPart") + std::to_string(i));
        clients[i] = participant_clients[i]->create_async_multiservice_client<test::TestDataType, test::TestDataType>(
            "test_topic");
        clients[i]->run(solution_listener);

        // Send N messages
        for (unsigned int j=0; j<test::N_MESSAGES; j++)
        {
            // Get new task
            std::string task_str = std::string("NEW_TASK_") + std::to_string(i) + "_" + std::to_string(j);

            // Send task
            clients[i]->send_request_async(std::make_shared<test::TestDataType>(task_str));
        }
    }

    // Create Participants and Servers
    std::unique_ptr<Participant> participant_server = std::make_unique<Participant>("ServerTestPart");
    std::shared_ptr<eprosima::amlip::dds::AsyncMultiServiceServer<test::TestDataType, test::TestDataType>> server =
        participant_server->create_async_multiservice_server<test::TestDataType, test::TestDataType>("test_topic");
    server->run(task_listener);

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * test::N_CLIENTS);

    // Send all messages again
    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        // Send N messages
        for (unsigned int j=0; j<test::N_MESSAGES; j++)
        {
            // Get new task
            std::string task_str = std::string("NEW_TASK_") + std::to_string(i) + "_" + std::to_string(j + test::N_MESSAGES);

            // Send task
            clients[i]->send_request_async(std::make_shared<test::TestDataType>(task_str));
        }
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * test::N_CLIENTS * 2);

    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        std::lock_guard<test::SolutionsReceivedType> guard(solution_listener->solutions);
        for (unsigned int j=0; j<test::N_MESSAGES * 2; j++)
        {
            std::string expected_str = std::string("new_task_") + std::to_string(i) + "_" + std::to_string(j);

            std::pair<types::TaskId, types::AmlipIdDataType> p = {j, participant_clients[i]->id()};
            ASSERT_EQ(solution_listener->solutions[p].first->to_string(), expected_str);
        }
    }
}

/**
 * Communicate N Asyn MS Client with M Async MS Server from different participants
 * They use TestDataType as data type for Data and Solution and they are created as strings and processed as to_lower.
 * STEPS:
 * - Create entities and run client
 * - Send N messages from client with server no running
 * - Run server
 * - Wait for client to get all answers
 * - Send N messages again
 * - Wait for client to get all answers
 * - Check all messages received
 */
TEST(asyncMultiServiceTest, communicate_service_n_to_n)
{
    // Create listeners for each entity
    std::shared_ptr<test::TestSolutionListener> solution_listener = std::make_shared<test::TestSolutionListener>();
    std::shared_ptr<test::TestTaskListener> task_listener = std::make_shared<test::TestTaskListener>();

    // Create Participants and Clients
    std::array<std::unique_ptr<Participant>, test::N_CLIENTS> participant_clients;
    std::array<
        std::shared_ptr<eprosima::amlip::dds::AsyncMultiServiceClient<test::TestDataType, test::TestDataType>>,
        test::N_CLIENTS
    > clients;

    // Create clients and run them sending N messages each
    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        participant_clients[i] = std::make_unique<Participant>(std::string("ClientTestPart") + std::to_string(i));
        clients[i] = participant_clients[i]->create_async_multiservice_client<test::TestDataType, test::TestDataType>(
            "test_topic");
        clients[i]->run(solution_listener);

        // Send N messages
        for (unsigned int j=0; j<test::N_MESSAGES; j++)
        {
            // Get new task
            std::string task_str = std::string("NEW_TASK_") + std::to_string(i) + "_" + std::to_string(j);

            // Send task
            clients[i]->send_request_async(std::make_shared<test::TestDataType>(task_str));
        }
    }

    // Create Participants and Servers
    std::array<std::unique_ptr<Participant>, test::N_SERVERS> participant_servers;
    std::array<
        std::shared_ptr<eprosima::amlip::dds::AsyncMultiServiceServer<test::TestDataType, test::TestDataType>>,
        test::N_SERVERS
    > servers;

    // Create servers and run them sending N messages each
    for (unsigned int i=0; i<test::N_SERVERS; i++)
    {
        participant_servers[i] = std::make_unique<Participant>(std::string("ServerTestPart") + std::to_string(i));
        servers[i] = participant_servers[i]->create_async_multiservice_server<test::TestDataType, test::TestDataType>(
            "test_topic");
        servers[i]->run(task_listener);
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * test::N_CLIENTS);

    // Send all messages again
    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        // Send N messages
        for (unsigned int j=0; j<test::N_MESSAGES; j++)
        {
            // Get new task
            std::string task_str = std::string("NEW_TASK_") + std::to_string(i) + "_" + std::to_string(j + test::N_MESSAGES);

            // Send task
            clients[i]->send_request_async(std::make_shared<test::TestDataType>(task_str));
        }
    }

    // Wait for all tasks answered
    solution_listener->waiter.wait_equal(test::N_MESSAGES * test::N_CLIENTS * 2);

    for (unsigned int i=0; i<test::N_CLIENTS; i++)
    {
        std::lock_guard<test::SolutionsReceivedType> guard(solution_listener->solutions);
        for (unsigned int j=0; j<test::N_MESSAGES * 2; j++)
        {
            std::string expected_str = std::string("new_task_") + std::to_string(i) + "_" + std::to_string(j);

            std::pair<types::TaskId, types::AmlipIdDataType> p = {j, participant_clients[i]->id()};
            ASSERT_EQ(solution_listener->solutions[p].first->to_string(), expected_str);
        }
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);
    return RUN_ALL_TESTS();
}
