// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <fstream>

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <fastdds/rtps/common/SerializedPayload.h>

#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

using namespace eprosima::amlip::types;
using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;

/**
 * Test \c ModelStatisticsDataType construction
 *
 */
TEST(modelStatisticsTest, create_statistics)
{
    std::string name("TestNode");
    std::string data_str("TestData");
    std::vector<ByteType> data_vector;

    for (char c : data_str)
    {
        data_vector.push_back(static_cast<ByteType>(c));
    }

    ModelStatisticsDataType statistics_1;
    ModelStatisticsDataType statistics_2(name);
    ModelStatisticsDataType statistics_3(name, data_str);
    ModelStatisticsDataType statistics_4(name, data_str, true);
    ModelStatisticsDataType statistics_5(name, data_str, false);
    ModelStatisticsDataType statistics_6(name, data_vector);
    ModelStatisticsDataType statistics_7(name, data_vector, true);
    ModelStatisticsDataType statistics_8(name, data_vector, false);

    ASSERT_EQ(statistics_1.name(), "ModelStatisticsDataTypeName");
    ASSERT_EQ(statistics_1.data(), nullptr);
    ASSERT_EQ(statistics_1.data_size(), 0);

    ASSERT_EQ(statistics_2.name(), name);
    ASSERT_EQ(statistics_2.data(), nullptr);
    ASSERT_EQ(statistics_2.data_size(), 0);

    ASSERT_EQ(statistics_3.name(), name);
    ASSERT_EQ(std::string((char*)statistics_3.data()), data_str);
    ASSERT_EQ(statistics_3.data_size(), data_str.size());

    ASSERT_EQ(statistics_4.name(), name);
    ASSERT_EQ(std::string((char*)statistics_4.data()), data_str);
    ASSERT_EQ(statistics_4.data_size(), data_str.size());

    ASSERT_EQ(statistics_5.name(), name);
    ASSERT_EQ(std::string((char*)statistics_5.data()), data_str);
    ASSERT_EQ(statistics_5.data_size(), data_str.size());

    ASSERT_EQ(statistics_6.name(), name);
    ASSERT_EQ(std::string((char*)statistics_6.data()), data_str);
    ASSERT_EQ(statistics_6.data_size(), data_str.size());

    ASSERT_EQ(statistics_7.name(), name);
    ASSERT_EQ(std::string((char*)statistics_7.data()), data_str);
    ASSERT_EQ(statistics_7.data_size(), data_str.size());

    ASSERT_EQ(statistics_8.name(), name);
    ASSERT_EQ(std::string((char*)statistics_8.data()), data_str);
    ASSERT_EQ(statistics_8.data_size(), data_str.size());
}

/**
 * Test \c ModelStatisticsDataType data_
 *
 */
TEST(modelStatisticsTest, statistics_with_long_data)
{
    std::string name("TestNode");
    std::string data_str;

    std::ifstream file("resources/el_quijote.txt");
    if (file.is_open())
    {
        data_str = std::string((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
    }
    else
    {
        throw std::runtime_error("Failed to open file: resources/el_quijote.txt");
    }

    ModelStatisticsDataType statistics_1(name, data_str);

    ASSERT_EQ(statistics_1.name(), name);
    ASSERT_EQ(statistics_1.to_string(), data_str);
    ASSERT_EQ(statistics_1.data_size(), data_str.length());

    std::vector<ByteType> data_vector;

    for (char c : data_str)
    {
        data_vector.push_back(static_cast<ByteType>(c));
    }

    ModelStatisticsDataType statistics_2(name, data_vector);

    ASSERT_EQ(statistics_2.name(), name);
    ASSERT_EQ(statistics_2.to_vector(), data_vector);
    ASSERT_EQ(statistics_2.data_size(), data_vector.size());
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
