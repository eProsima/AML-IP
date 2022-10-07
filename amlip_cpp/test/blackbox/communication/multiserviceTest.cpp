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

#include <dds/Participant.hpp>
#include <types/AmlipIdDataType.hpp>

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

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
