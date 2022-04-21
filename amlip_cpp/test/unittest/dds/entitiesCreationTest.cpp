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

using namespace eprosima::amlip::dds;
using namespace eprosima::amlip;

/**
 * Create participant
 */
TEST(EntitiesCreationTest, create_participant)
{
    Participant participant_(std::string("TestParticipant"));
}

/**
 * Create writer
 */
TEST(EntitiesCreationTest, create_writer)
{
    Participant participant(std::string("TestWriterParticipant"));

    std::shared_ptr<Writer<types::AmlipIdDataType>> entity_ =
        participant.create_writer<types::AmlipIdDataType>("test_topic");
}

/**
 * Create reader
 */
TEST(EntitiesCreationTest, create_reader)
{
    Participant participant(std::string("TestReaderParticipant"));

    std::shared_ptr<Reader<types::AmlipIdDataType>> entity_ =
        participant.create_reader<types::AmlipIdDataType>("test_topic");
}

/**
 * Create directwriter
 */
TEST(EntitiesCreationTest, create_directwriter)
{
    Participant participant(std::string("TestDirectWriterParticipant"));

    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> entity_ =
        participant.create_direct_writer<types::AmlipIdDataType>("test_topic");
}

/**
 * Create targetedreader
 */
TEST(EntitiesCreationTest, create_targetedreader)
{
    Participant participant(std::string("TestTargetedReaderParticipant"));

    std::shared_ptr<TargetedReader<types::AmlipIdDataType>> entity_ =
        participant.create_targeted_reader<types::AmlipIdDataType>("test_topic");
}

/**
 * Create MS Client
 */
TEST(EntitiesCreationTest, create_ms_client)
{
    Participant participant(std::string("TestMsClientParticipant"));

    std::shared_ptr<MultiServiceClient<types::AmlipIdDataType, types::AmlipIdDataType>>  entity_ =
        participant.create_multiservice_client<types::AmlipIdDataType, types::AmlipIdDataType>("test_topic");

    // TODO remove
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

/**
 * Create MS Server
 */
TEST(EntitiesCreationTest, create_ms_server)
{
    Participant participant(std::string("TestMsServerParticipant"));

    std::shared_ptr<MultiServiceServer<types::AmlipIdDataType, types::AmlipIdDataType>>  entity_ =
        participant.create_multiservice_server<types::AmlipIdDataType, types::AmlipIdDataType>("test_topic");

    // TODO remove
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
