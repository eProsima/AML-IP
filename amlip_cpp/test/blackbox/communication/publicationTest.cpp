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

#include <dds/Participant.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace test {

constexpr const uint32_t N_ITERATIONS_TEST = 5;

eprosima::fastdds::dds::DataWriterQos writer_qos()
{
    eprosima::fastdds::dds::DataWriterQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

    return qos;
}

eprosima::fastdds::dds::DataReaderQos reader_qos()
{
    eprosima::fastdds::dds::DataReaderQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

    return qos;
}

} /* namespace test */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

using namespace eprosima::amlip;
using namespace eprosima::amlip::dds;

/**
 * Create a Participant, from there create a Writer and a Reader and communicate them.
 * It will use the AmlipIdDataType as data type.
 */
TEST(PublicationTest, communicate_reader_writer)
{
    // Create a participant
    Participant participant(std::string("1_TestParticipant"));

    // Create a writer
    std::shared_ptr<Writer<types::AmlipIdDataType>> writer =
            participant.create_writer<types::AmlipIdDataType>("TestTopic_1", test::writer_qos());

    // Create a reader
    std::shared_ptr<Reader<types::AmlipIdDataType>> reader =
            participant.create_reader<types::AmlipIdDataType>("TestTopic_1", test::reader_qos());


    // Create a data
    types::AmlipIdDataType to_send_data(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {1});

    // Write data
    writer->publish(to_send_data);

    // Wait for data to arrive
    reader->wait_data_available();
    types::AmlipIdDataType received_data = reader->read();

    // Check data
    ASSERT_EQ(to_send_data, received_data);
}

/**
 * Create a Participant, from there create a Writer and a Reader and communicate them.
 * It will use the AmlipIdDataType as data type.
 */
TEST(PublicationTest, communicate_reader_writer_multiple_messages)
{
    // Create a participant
    Participant participant(std::string("1_1_TestParticipant"));

    // Create a writer
    std::shared_ptr<Writer<types::AmlipIdDataType>> writer =
            participant.create_writer<types::AmlipIdDataType>("TestTopic_1_1", test::writer_qos());

    // Create a reader
    std::shared_ptr<Reader<types::AmlipIdDataType>> reader =
            participant.create_reader<types::AmlipIdDataType>("TestTopic_1_1", test::reader_qos());

    // Create and send N data
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        types::AmlipIdDataType to_send_data(
            {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
            {static_cast<uint8_t>(i)});

        // Write data
        writer->publish(to_send_data);
    }

    // Store data received and check it comes from every writer
    std::vector<types::AmlipIdDataType> received_data;

    // Wait for data to arrive
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        reader->wait_data_available();
        received_data.push_back(reader->read());
    }

    // Check data
    ASSERT_EQ(received_data.size(), test::N_ITERATIONS_TEST);

    // Check data comes from every writer
    std::vector<bool> id_received(test::N_ITERATIONS_TEST);
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        id_received[received_data[i].id()[0]] = true;
    }

    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        ASSERT_TRUE(id_received[received_data[i].id()[0]]);
    }
}

/**
 * Create a Participant, from there create N Writers and a Reader and communicate them.
 * It will use the AmlipIdDataType as data type.
 */
TEST(PublicationTest, communicate_reader_multiple_writers)
{
    // Create a participant
    Participant participant(std::string("2_TestParticipant"));

    // Create writers
    std::vector<std::shared_ptr<Writer<types::AmlipIdDataType>>> writers;
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        writers.push_back(
            participant.create_writer<types::AmlipIdDataType>("TestTopic_2", test::writer_qos()));
    }

    // Create a reader
    std::shared_ptr<Reader<types::AmlipIdDataType>> reader =
            participant.create_reader<types::AmlipIdDataType>("TestTopic_2", test::reader_qos());

    // Write data, once from each writer
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        types::AmlipIdDataType to_send_data(
            {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
            {static_cast<uint8_t>(i)});
        writers[i]->publish(to_send_data);
    }

    // Store data received and check it comes from every writer
    std::vector<types::AmlipIdDataType> received_data;

    // Wait for data to arrive
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        reader->wait_data_available();
        received_data.push_back(reader->read());
    }

    // Check data
    ASSERT_EQ(received_data.size(), test::N_ITERATIONS_TEST);

    // Check data comes from every writer
    std::vector<bool> id_received(test::N_ITERATIONS_TEST);
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        id_received[received_data[i].id()[0]] = true;
    }

    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        ASSERT_TRUE(id_received[received_data[i].id()[0]]);
    }
}

/**
 * Create a Participant, from there create N Readers and a Writer and communicate them.
 * It will use the AmlipIdDataType as data type.
 */
TEST(PublicationTest, communicate_multiple_readers_writer)
{
    // Create a participant
    Participant participant(std::string("3_TestParticipant"));

    // Create a writer
    std::shared_ptr<Writer<types::AmlipIdDataType>> writer =
            participant.create_writer<types::AmlipIdDataType>("TestTopic_3", test::writer_qos());

    // Create a reader
    std::vector<std::shared_ptr<Reader<types::AmlipIdDataType>>> readers;
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        readers.push_back(
            participant.create_reader<types::AmlipIdDataType>("TestTopic_3", test::reader_qos()));
    }

    // Write 1 data
    types::AmlipIdDataType to_send_data(
        {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
        {3});
    writer->publish(to_send_data);

    // Wait for data to arrive in each Reader
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        readers[i]->wait_data_available();
        types::AmlipIdDataType received_data = readers[i]->read();

        // Checks data
        ASSERT_EQ(to_send_data, received_data);
    }
}

/**
 * Create a Participant, from there create N Writers and N Readers and communicate them.
 * It will use the AmlipIdDataType as data type.
 * It will only check that each reader receives N data, it will not check the data internally to simplify
 */
TEST(PublicationTest, communicate_multiple_readers_multiple_writers)
{
    // Create a participant
    Participant participant(std::string("4_TestParticipant"));

    // Create writers
    std::vector<std::shared_ptr<Writer<types::AmlipIdDataType>>> writers;
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        writers.push_back(
            participant.create_writer<types::AmlipIdDataType>("TestTopic_4", test::writer_qos()));
    }

    // Create a reader
    std::vector<std::shared_ptr<Reader<types::AmlipIdDataType>>> readers;
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        readers.push_back(
            participant.create_reader<types::AmlipIdDataType>("TestTopic_4", test::reader_qos()));
    }

    // Write data, once from each writer
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        types::AmlipIdDataType to_send_data(
            {'T', 'e', 's', 't', 'D', 'a', 't', 'a'},
            {static_cast<uint8_t>(i)});
        writers[i]->publish(to_send_data);
    }

    // Store data received and check it comes from every writer
    std::vector<types::AmlipIdDataType> received_data;

    // Wait for data to arrive in each reader
    for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
    {
        for (uint32_t i = 0; i < test::N_ITERATIONS_TEST; ++i)
        {
            readers[i]->wait_data_available();
            received_data.push_back(readers[i]->read());
        }
    }

    // Check len of data received
    ASSERT_EQ(received_data.size(), test::N_ITERATIONS_TEST* test::N_ITERATIONS_TEST);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
