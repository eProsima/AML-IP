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
 * @file Reader.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_TARGETEDREADER_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_TARGETEDREADER_IPP

#include <dds/network_utils/direct_write.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
TargetedReader<T>::TargetedReader(
        const types::AmlipIdDataType own_id,
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataReaderQos qos /* = Reader::default_datareader_qos() */)
    : Reader<T>(utils::direct_topic(topic, own_id), dds_handler, qos)
{
}

template <typename T>
eprosima::fastdds::dds::DataReaderQos TargetedReader<T>::default_targetedreader_qos()
{
    eprosima::fastdds::dds::DataReaderQos qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;

    // Preallocated with realloc
    qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

    return qos;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_TARGETEDREADER_IPP */
