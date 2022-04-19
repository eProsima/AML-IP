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
#include <types/AmlipIdDataType.hpp>

using namespace eprosima::amlip;
using namespace eprosima::amlip::dds;

/**
 * Create a Participant, from there create a Writer and a Reader and communicate them.
 * It will use the AmlipId as data type.
 */
TEST(ReaderWriterTest, communicate_reader_writer)
{
    // Create a participant
    Participant participant("TestParticipant");

    // Create a writer
    std::shared_ptr<Writer<types::AmlipIdDataType>> writer =
        participant.create_writer<types::AmlipIdDataType>("TestTopic");

    // Create a reader
    std::shared_ptr<Reader<types::AmlipIdDataType>> reader =
        participant.create_reader<types::AmlipIdDataType>("TestTopic");


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

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
