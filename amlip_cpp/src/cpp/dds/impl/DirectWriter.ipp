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
 * @file DirectWriter.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_IMPL_DIRECTWRITER_IPP
#define AMLIPCPP__SRC_CPP_DDS_IMPL_DIRECTWRITER_IPP

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/direct_write.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
DirectWriter<T>::DirectWriter(
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataWriterQos qos /* = DirectWriter::default_directwriter_qos() */)
    : topic_(topic)
    , dds_handler_(dds_handler)
    , qos_(qos)
{
}

template <typename T>
DirectWriter<T>::~DirectWriter()
{
    // DataWriters will be erased by themselves
}

template <typename T>
eprosima::fastrtps::types::ReturnCode_t DirectWriter<T>::write(
        const types::AmlipIdDataType& target_id,
        T& data)
{
    std::shared_ptr<Writer<T>> target_writer =
            get_target_writer_(target_id);

    return target_writer->publish(data);
}

template <typename T>
eprosima::utils::event::AwakeReason DirectWriter<T>::wait_match(
        const types::AmlipIdDataType& target_id,
        const eprosima::utils::Duration_ms& timeout /* = 0 */)
{
    // Get writer associated with this target (if it does not exist it creates it)
    std::shared_ptr<Writer<T>> target_writer = get_target_writer_(target_id);

    // Do wait in listener
    return target_writer->wait_match(timeout);
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos DirectWriter<T>::default_directwriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos qos = utils::default_datawriter_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

    return qos;
}

template <typename T>
std::shared_ptr<Writer<T>> DirectWriter<T>::get_target_writer_(
        types::AmlipIdDataType target_id)
{
    auto it = writers_.find(target_id);

    if (it == writers_.end())
    {
        // Create new writer
        std::shared_ptr<Writer<T>> new_writer = std::make_shared<Writer<T>>(
            utils::direct_topic(topic_, target_id), dds_handler_, qos_);

        writers_.emplace(target_id, new_writer);

        return new_writer;
    }
    else
    {
        return it->second;
    }
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_IMPL_DIRECTWRITER_IPP */
