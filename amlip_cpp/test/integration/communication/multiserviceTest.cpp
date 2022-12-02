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

#include <dds/Participant.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/GenericDataType.hpp>

namespace test {

constexpr const uint32_t N_SERVERS = 4;
constexpr const uint32_t N_CLIENTS = 8;
constexpr const uint32_t N_MESSAGES = 20;
// Warning: N_CLIENTS * N_MESSAGES % N_SERVERS must be 0

class TestDataType : public eprosima::amlip::types::GenericDataType
{
    using GenericDataType::GenericDataType;
};

} // namespace test

using namespace eprosima::amlip;
using namespace eprosima::amlip::dds;

/**
 * Communicate a MS Client and a MS Server from different participants
 * They use AmlipIdDataType as data type for Data and Solution.
 */
TEST(MultiServiceTest, communicate_service_one_on_one)
{
    // Create participants
    Participant participant_client(std::string("ClientTestPart"));
    Participant participant_server(std::string("ServerTestPart"));

    // Create a MS Client
    std::shared_ptr<MultiServiceClient<types::AmlipIdDataType, types::AmlipIdDataType>> client =
            participant_client.create_multiservice_client<types::AmlipIdDataType, types::AmlipIdDataType>("test_topic");

    // Create a MS Server
    std::shared_ptr<MultiServiceServer<types::AmlipIdDataType, types::AmlipIdDataType>> server =
            participant_server.create_multiservice_server<types::AmlipIdDataType, types::AmlipIdDataType>("test_topic");

    // Create a data
    types::AmlipIdDataType to_send_data(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {1});

    // Get id of client to check in server after answering
    types::AmlipIdDataType client_id = participant_client.id();

    // Receive data from server and answer solution in another thread
    // Response will be to add 1 to id given
    std::thread server_thread(
        [server, &client_id]()
        {
            types::MsReferenceDataType task_solved_reference = server->process_task_sync(
                []
                    (const types::AmlipIdDataType& data)
                {
                    auto id_received = data.id();
                    id_received[0] += 1;
                    return types::AmlipIdDataType(data.base64_name(), id_received);
                });
            ASSERT_EQ(task_solved_reference.client_id(), client_id);
        });

    // Send data from client to server
    types::AmlipIdDataType solution = client->send_request_sync(to_send_data);

    // Check data received
    ASSERT_EQ(to_send_data.name(), solution.name());
    ASSERT_EQ(to_send_data.id()[0] + 1, solution.id()[0]);

    // Wait for server thread to finish
    server_thread.join();
}

/**
 * Communicate N MSClients with M MSServers in the same topic.
 * Each entity will work in its own thread.
 * The data will be a string and the process will be to lowercase it.
 */
TEST(MultiServiceTest, communicate_service_n_to_n)
{

    // Each Client Routine
    auto client_lambda = [](int id, int tasks_to_send){

        // Create entities
        Participant participant(std::string("ClientTestPart") + std::to_string(id));
        auto client = participant.create_multiservice_client<test::TestDataType, test::TestDataType>("test_topic");

        // Wait for N messages
        while (tasks_to_send--)
        {
            // Get new task
            std::string task_str = std::string("NEW_TASK_") + std::to_string(id) + std::string("_") + std::to_string(tasks_to_send);
            std::string solution_expected = std::string("new_task_") + std::to_string(id) + std::string("_") + std::to_string(tasks_to_send);
            test::TestDataType task(task_str);

            // Send task
            auto solution = client->send_request_sync(task);

            // Check solution
            ASSERT_EQ(solution.to_string(), solution_expected);
        }
    };

    // Each Server Routine
    auto server_lambda = [](int id, int tasks_to_process){

        // Create entities
        Participant participant(std::string("ServerTestPart") + std::to_string(id));
        auto server = participant.create_multiservice_server<test::TestDataType, test::TestDataType>("test_topic");

        // Create processing routine -> to lowercase
        auto processing_routine = [](const test::TestDataType& task)
        {
            std::string result = task.to_string();
            eprosima::utils::to_lowercase(result);
            return test::TestDataType(result);
        };

        // Wait for N messages
        while (tasks_to_process--)
        {
            // Process task and convert it to lowercase
            server->process_task_sync(processing_routine);
        }
    };

    // Generate all clients
    std::array<std::thread, test::N_CLIENTS> clients_threads;
    for (unsigned int i=0; i< test::N_CLIENTS; i++)
    {
        clients_threads[i] = std::thread(client_lambda, i, test::N_MESSAGES);
    }

    // Generate all servers
    auto messages_per_server = test::N_MESSAGES * test::N_CLIENTS / test::N_SERVERS;
    std::array<std::thread, test::N_SERVERS> servers_threads;
    for (unsigned int i=0; i< test::N_SERVERS; i++)
    {
        servers_threads[i] = std::thread(server_lambda, i, messages_per_server);
    }

    // Wait for all of them to finish
    for (unsigned int i=0; i< test::N_CLIENTS; i++)
    {
        clients_threads[i].join();
    }
    for (unsigned int i=0; i< test::N_SERVERS; i++)
    {
        servers_threads[i].join();
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
