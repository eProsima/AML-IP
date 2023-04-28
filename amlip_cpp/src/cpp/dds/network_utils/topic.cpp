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

/**
 * @file topic.cpp
 */

#include <dds/network_utils/topic.hpp>
#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

eprosima::fastdds::dds::DataWriterQos status_writer_qos() noexcept
{
    eprosima::fastdds::dds::DataWriterQos qos = default_datawriter_qos();

    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    // Not needed to use REALLOC policy
    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_MEMORY_MODE;

    return qos;
}

eprosima::fastdds::dds::DataReaderQos status_reader_qos() noexcept
{
    eprosima::fastdds::dds::DataReaderQos qos = default_datareader_qos();

    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    // TODO: add key to StatusType and use KEEP_LAST 1 as the key will allow to handle it

    // Not needed to use REALLOC policy
    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_MEMORY_MODE;

    return qos;
}

eprosima::fastdds::dds::DataWriterQos model_writer_qos() noexcept
{
    eprosima::fastdds::dds::DataWriterQos qos = default_datawriter_qos();

    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    return qos;
}

eprosima::fastdds::dds::DataReaderQos model_reader_qos() noexcept
{
    eprosima::fastdds::dds::DataReaderQos qos = default_datareader_qos();

    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    return qos;
}

std::string type_name_mangling(
        const std::string& type_name) noexcept
{
    return std::string(TYPE_NAME_MANGLING) + type_name;
}

std::string topic_name_mangling(
        const std::string& topic_name) noexcept
{
    return std::string(TOPIC_NAME_MANGLING) + topic_name;
}

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
