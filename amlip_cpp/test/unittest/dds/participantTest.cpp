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

constexpr const uint32_t TEST_ITERATIONS = 5;

/**
 * Create a Participant and do nothing
 *
 * @note this test will fail if it breaks
 */
TEST(participantTest, create_dummy_participant)
{
    {
        eprosima::amlip::types::AmlipIdDataType id;
        eprosima::amlip::dds::Participant participant_(id);
    }
}

/**
 * Create a Participant with specific ids and check id is the correct one
 */
TEST(participantTest, id)
{
    for (uint32_t i = 0; i < TEST_ITERATIONS; ++i)
    {
        eprosima::amlip::types::AmlipIdDataType id(std::to_string(i));
        eprosima::amlip::dds::Participant participant(id);
        EXPECT_EQ(id, participant.id());
    }
}

/**
 * Create a Participant with specific ids and check name is the correct one given in id
 */
TEST(participantTest, get_name)
{
    for (uint32_t i = 0; i < TEST_ITERATIONS; ++i)
    {
        eprosima::amlip::types::AmlipIdDataType id(std::to_string(i));
        eprosima::amlip::dds::Participant participant(id);
        EXPECT_EQ(id.name(), participant.name());
    }
}

/**
 * Create a Participant with specific ids and check name is the correct one given in id
 */
TEST(participantTest, create_with_name)
{
    for (uint32_t i = 0; i < TEST_ITERATIONS; ++i)
    {
        std::string name("PartName" + std::to_string(i));
        eprosima::amlip::dds::Participant participant(name);
        EXPECT_EQ(name, participant.name());
    }
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
