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
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace test {

constexpr const uint32_t N_ITERATIONS_TEST = 5;

} /* namespace test */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

using namespace eprosima::amlip;
using namespace eprosima::amlip::dds;

/**
 * Create a Participant, from there create a DirectWriter and a TargetedReader and send a message from one to the other.
 * It will use the AmlipIdDataType as data type.
 */
TEST(DirectMessageTest, communicate_writer_reader)
{
    // Ids used along the test
    types::AmlipIdDataType client_id("SourceId1");
    types::AmlipIdDataType server_id("TargetId1");

    // Create 2 participants so they have different ids
    // Create a participant
    Participant client_participant(client_id);
    Participant target_participant(server_id);

    // Create a writer
    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> writer =
            client_participant.create_direct_writer<types::AmlipIdDataType>("TestTopic_1");

    // Create a reader
    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> reader =
            target_participant.create_targeted_reader<types::AmlipIdDataType>("TestTopic_1");

    // Create a data
    // TODO: refactor for any number of RAND_SIZE in AmlipId
    types::AmlipIdDataType to_send_data(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {1});

    // Write data
    writer->write(server_id, to_send_data);

    // Wait for data to arrive
    reader->wait_data_available();
    types::AmlipIdDataType received_data = reader->read();

    // Check data
    ASSERT_EQ(to_send_data, received_data);
}

/**
 * Create a Participant, from there create a DirectWriter and a TargetedReader and send multiple messages from one to the other.
 * It will use the AmlipIdDataType as data type.
 */
TEST(DirectMessageTest, communicate_writer_reader_multiple_messages)
{
    // Ids used along the test
    types::AmlipIdDataType client_id("SourceId2");
    types::AmlipIdDataType server_id("TargetId2");

    // Create 2 participants so they have different ids
    // Create a participant
    Participant client_participant(client_id);
    Participant target_participant(server_id);

    // Create a writer
    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> writer =
            client_participant.create_direct_writer<types::AmlipIdDataType>("TestTopic_2");

    // Create a reader
    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> reader =
            target_participant.create_targeted_reader<types::AmlipIdDataType>("TestTopic_2");

    // Create a data
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        types::AmlipIdDataType to_send_data(
            {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
            {static_cast<uint8_t>(i)});

        // Write data
        writer->write(server_id, to_send_data);
    }

    // Store the sum of ids got
    uint32_t ids_checksum = 0;

    // Wait for data to arrive
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        reader->wait_data_available();
        types::AmlipIdDataType received_data = reader->read();
        ids_checksum += received_data.id()[0];
    }

    // Check the sum of ids is correct
    ASSERT_EQ(ids_checksum, test::N_ITERATIONS_TEST* (test::N_ITERATIONS_TEST - 1) / 2);
}

/**
 * Create a Participant, from there create 2 DirectWriters and a TargetedReader and send a message from each one
 * It will use the AmlipIdDataType as data type.
 */
TEST(DirectMessageTest, communicate_multiple_writers_reader)
{
    // Ids used along the test
    types::AmlipIdDataType client_id_1("SourceId3");
    types::AmlipIdDataType client_id_2("SourceId3");
    types::AmlipIdDataType server_id("TargetId3");

    // Create 2 participants so they have different ids
    // Create a participant
    Participant client_participant_1(client_id_1);
    Participant client_participant_2(client_id_2);
    Participant target_participant(server_id);

    // Create a writer 1
    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> writer_1 =
            client_participant_1.create_direct_writer<types::AmlipIdDataType>("TestTopic_3");

    // Create a writer 2
    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> writer_2 =
            client_participant_2.create_direct_writer<types::AmlipIdDataType>("TestTopic_3");

    // Create a reader
    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> reader =
            target_participant.create_targeted_reader<types::AmlipIdDataType>("TestTopic_3");

    // Create data
    types::AmlipIdDataType to_send_data_1(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {1});
    types::AmlipIdDataType to_send_data_2(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {2});

    // Send data from writer
    writer_1->write(server_id, to_send_data_1);
    writer_2->write(server_id, to_send_data_2);

    // Store the sum of ids got
    uint32_t ids_checksum = 0;

    // Wait for data to arrive
    for (uint32_t i = 0; i < 2; ++i)
    {
        reader->wait_data_available();
        types::AmlipIdDataType received_data = reader->read();
        ids_checksum += received_data.id()[0];
    }

    // Check the sum of ids is correct
    ASSERT_EQ(ids_checksum, 3u);
}

/**
 * Create a Participant, from there create 2 DirectWriters and a TargetedReader and send a message from each one
 * It will use the AmlipIdDataType as data type.
 */
TEST(DirectMessageTest, communicate_writer_multiple_readers)
{
    // Ids used along the test
    types::AmlipIdDataType client_id("SourceId4");
    types::AmlipIdDataType server_id_1("TargetId4");
    types::AmlipIdDataType server_id_2("TargetId4");

    // Create 2 participants so they have different ids
    // Create a participant
    Participant client_participant(client_id);
    Participant target_participant_1(server_id_1);
    Participant target_participant_2(server_id_2);

    // Create a writer
    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> writer =
            client_participant.create_direct_writer<types::AmlipIdDataType>("TestTopic_4");

    // Create a reader 1
    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> reader_1 =
            target_participant_1.create_targeted_reader<types::AmlipIdDataType>("TestTopic_4");

    // Create a reader 2
    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> reader_2 =
            target_participant_2.create_targeted_reader<types::AmlipIdDataType>("TestTopic_4");

    // Create data
    types::AmlipIdDataType to_send_data(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {4});

    // Write to reader 1
    writer->write(server_id_1, to_send_data);

    reader_1->wait_data_available();
    types::AmlipIdDataType received_data_1 = reader_1->read();
    ASSERT_EQ(to_send_data, received_data_1);

    // Check the other Reader has not received any data
    ASSERT_FALSE(reader_2->is_data_available());

    // Write to reader 2
    writer->write(server_id_2, to_send_data);

    reader_2->wait_data_available();
    types::AmlipIdDataType received_data_2 = reader_2->read();
    ASSERT_EQ(to_send_data, received_data_2);

    // Check the other Reader has not received any data
    ASSERT_FALSE(reader_1->is_data_available());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
