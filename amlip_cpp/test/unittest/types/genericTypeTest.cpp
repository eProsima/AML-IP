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
#include <types/AmlipIdDataType.hpp>
#include <types/GenericType.hpp>

using namespace eprosima::amlip::types;
using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;

template <class T>
void test_generic_serialization_deserialization(T data_to_send, size_t serialized_payload_size=64)
{
    GenericType generic_type(static_cast<void*>(&data_to_send), sizeof(data_to_send));
    AmlipGenericTopicDataType<GenericType> data_type;

    SerializedPayload_t payload = SerializedPayload_t(serialized_payload_size);

    data_type.serialize(static_cast<GenericType*>(&generic_type), &payload);

    GenericType new_generic_type;
    data_type.deserialize(&payload, static_cast<GenericType*>(&new_generic_type));
    T data_received = *(T*)new_generic_type.data();

    ASSERT_EQ(data_to_send, data_received);
}

/**
 * Test serialization and deserialization of an integer
 *
 */
TEST(genericTypeTest, generic_serialization_deserialization_int)
{
    test_generic_serialization_deserialization(8);
}

/**
 * Test serialization and deserialization of a string
 *
 */
TEST(genericTypeTest, generic_serialization_deserialization_str)
{
    test_generic_serialization_deserialization("Testing generic type");
}

/**
 * Test serialization and deserialization of a AmlipDataType object
 *
 */
TEST(genericTypeTest, generic_serialization_deserialization_amlipIdDataType)
{
    test_generic_serialization_deserialization(AmlipIdDataType("TestNode"));
}


int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
