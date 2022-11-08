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
 * @file dds_qos.cpp
 */

#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

fastdds::dds::DomainParticipantQos default_domain_participant_qos()
{
    eprosima::fastdds::dds::DomainParticipantQos qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    return qos;
}

fastdds::dds::PublisherQos default_publisher_qos()
{
    return fastdds::dds::PUBLISHER_QOS_DEFAULT;
}

fastdds::dds::DataWriterQos default_datawriter_qos()
{
    fastdds::dds::DataWriterQos qos;

    // Preallocated with realloc
    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    qos.publish_mode().kind = eprosima::fastdds::dds::PublishModeQosPolicyKind::SYNCHRONOUS_PUBLISH_MODE;

    // Disabling datasharing
    qos.data_sharing().off();

    return qos;
}

fastdds::dds::SubscriberQos default_subscriber_qos()
{
    return fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
}

fastdds::dds::DataReaderQos default_datareader_qos()
{
    fastdds::dds::DataReaderQos qos;

    // Preallocated with realloc
    qos.endpoint().history_memory_policy =
            eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    // Disabling datasharing
    qos.data_sharing().off();

    return qos;
}

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
