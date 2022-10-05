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
    Participant participant_("TestParticipant");
}

/**
 * Create writer
 */
TEST(EntitiesCreationTest, create_writer)
{
    Participant participant("TestWriterParticipant");

    std::shared_ptr<Writer<types::AmlipIdDataType>> entity_ =
        participant.create_writer<types::AmlipIdDataType>("test_topic");
}

/**
 * Create reader
 */
TEST(EntitiesCreationTest, create_reader)
{
    Participant participant("TestReaderParticipant");

    std::shared_ptr<Reader<types::AmlipIdDataType>> entity_ =
        participant.create_reader<types::AmlipIdDataType>("test_topic");
}

/**
 * Create directwriter
 */
TEST(EntitiesCreationTest, create_directwriter)
{
    Participant participant("TestDirectWriterParticipant");

    std::shared_ptr<DirectWriter<types::AmlipIdDataType>> entity_ =
        participant.create_direct_writer<types::AmlipIdDataType>("test_topic");
}

/**
 * Create targetedreader
 */
TEST(EntitiesCreationTest, create_targetedreader)
{
    Participant participant("TestTargetedReaderParticipant");

    // std::shared_ptr<TargetedReader<types::AmlipIdDataType>> entity_ =
    //     participant.create_targeted_reader<types::AmlipIdDataType>("test_topic");

    auto entity_ =
        participant.create_targeted_reader<types::AmlipIdDataType>("test_topic");
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
