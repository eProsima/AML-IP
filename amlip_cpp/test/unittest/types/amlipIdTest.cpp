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

#include <fastdds/rtps/common/SerializedPayload.h>

#include <types/AmlipGenericTopicDataType.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

using namespace eprosima::amlip::types;
using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;

/**
 * Test \c AmlipIdDataType construction
 *
 */
TEST(amlipIdTest, create_id)
{
    std::string name("TestNode");
    AmlipIdDataType id = AmlipIdDataType::new_unique_id(name);

    ASSERT_TRUE(id.is_defined());
    ASSERT_EQ(id.name(), name.substr(0, eprosima::amlip::types::NAME_SIZE));
}

/**
 * Construct multiple \c AmlipIdDataType with same and different names, and check they are never (or hardly ever) equal
 *
 */
TEST(amlipIdTest, ids_not_equal)
{
    AmlipIdDataType id_A1 = AmlipIdDataType::new_unique_id("NodeA");
    AmlipIdDataType id_A2 = AmlipIdDataType::new_unique_id("NodeA");
    AmlipIdDataType id_B = AmlipIdDataType::new_unique_id("NodeB");

    ASSERT_NE(id_A1, id_A2);
    ASSERT_NE(id_A1, id_B);
    ASSERT_NE(id_A2, id_B);
}

/**
 * Test serialization and deserialization
 *
 */
TEST(amlipIdTest, serialization_deserialization)
{
    AmlipIdDataType id = AmlipIdDataType("TestNode");
    AmlipGenericTopicDataType<AmlipIdDataType> data_type;

    SerializedPayload_t payload = SerializedPayload_t(64);

    data_type.serialize(static_cast<AmlipIdDataType*>(&id), &payload);

    AmlipIdDataType new_id;
    data_type.deserialize(&payload, static_cast<AmlipIdDataType*>(&new_id));

    ASSERT_EQ(id, new_id);
}

int main(
        int argc,
        char** argv)
{
    // initialize the random number generator
    srand (time(NULL));

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
