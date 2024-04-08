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

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/GenericDataType.hpp>
#include <types/AmlipGenericTopicDataType.hpp>

#include <fastdds/rtps/common/SerializedPayload.h>

using namespace eprosima::amlip::types;
using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;

/*!
 * @brief This function receives a pointer to a stream of bytes and the size it occupies in memory, "casts" the input
 * to \c GenericDataType, and performs serialization + deserialization of serialization result.
 * @param bytes_to_serialize pointer to the stream of bytes to serialize and deserialize.
 * @param data_size size occupied by \c bytes_to_serialize in memory.
 */
GenericDataType generic_void_serialization_deserialization(
        void* bytes_to_serialize,
        size_t data_size)
{
    GenericDataType generic_type(bytes_to_serialize, data_size, true);
    AmlipGenericTopicDataType<GenericDataType> topic_data_type;

    // Payload to store serialized data
    SerializedPayload_t payload = SerializedPayload_t(topic_data_type.getSerializedSizeProvider(&generic_type)());

    // Serialize
    topic_data_type.serialize(&generic_type, &payload);

    // Deserialize
    GenericDataType deserialization_generic_type;
    topic_data_type.deserialize(&payload, &deserialization_generic_type);

    return deserialization_generic_type;
}

/**
 * Test serialization and deserialization of an integer.
 *
 */
TEST(genericDataTypeTest, generic_serialization_deserialization_int)
{
    uint32_t num = 8;

    GenericDataType deserialization_generic_type = generic_void_serialization_deserialization(static_cast<void*>(&num),
                    sizeof(uint32_t));

    ASSERT_EQ(num, *(uint32_t*)deserialization_generic_type.data());
}

/**
 * Test serialization and deserialization of a string.
 *
 */
TEST(genericDataTypeTest, generic_serialization_deserialization_str)
{
    std::string str("Testing generic type");

    GenericDataType deserialization_generic_type =
            generic_void_serialization_deserialization(static_cast<void*>((char*)str.c_str()), str.size());

    ASSERT_EQ(str, std::string((char*)deserialization_generic_type.data(), str.size()));
}

/**
 * Test serialization and deserialization of an \c AmlipIdDataType object.
 *
 * First, an \c AmlipIdDataType object is converted to a stream of bytes via serialization.

 * Next, the obtained serialized payload is "converted" to \c GenericDataType, being then serialized and deserialized
 * through \c generic_void_serialization_deserialization just like in previous tests.
 *
 * Finally, the result of previous deserialization is treated as an \c AmlipIdDataType serialized payload, which is
 * deserialized and compared with the original id object.
 */
TEST(genericDataTypeTest, generic_serialization_deserialization_amlipIdDataType)
{
    AmlipIdDataType id("TestNode");

    // Convert id's content to array of bytes via serialization
    AmlipGenericTopicDataType<AmlipIdDataType> topic_data_type;
    SerializedPayload_t payload = SerializedPayload_t(topic_data_type.getSerializedSizeProvider(&id)());
    topic_data_type.serialize(&id, &payload);

    // Serialize and deserialize array of bytes treated as generic data type
    GenericDataType deserialization_generic_type =
            generic_void_serialization_deserialization(static_cast<void*>(payload.data), payload.length);

    // Fill content of new id with stream of bytes obtained from deserialization
    AmlipIdDataType new_id;
    size_t data_size = deserialization_generic_type.data_size();
    SerializedPayload_t new_payload(data_size);
    new_payload.length = data_size;
    memcpy(new_payload.data, deserialization_generic_type.data(), data_size);
    topic_data_type.deserialize(&new_payload, &new_id);

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
