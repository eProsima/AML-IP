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
 * @file Writer.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_IMPL_WRITER_IPP
#define AMLIPCPP__SRC_CPP_DDS_IMPL_WRITER_IPP

#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Writer<T>::Writer(
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataWriterQos qos /* = Writer::default_datawriter_qos() */)
    : topic_(topic)
{
    auto dds_handler_locked = dds_handler.lock_with_exception();

    datawriter_ = dds_handler_locked->create_datawriter<T>(
        topic_,
        qos);

    datawriter_->set_listener(this);
}

template <typename T>
Writer<T>::~Writer()
{
    // Unsetting listener for datawriter, as the datawriter could be alive after this object has been destroyed
    // In case datawriter has already been destroyed, do nothing
    auto datawriter_locked = datawriter_.lock();

    if (datawriter_locked)
    {
        datawriter_locked->set_listener(nullptr);
    }
}

template <typename T>
eprosima::fastrtps::types::ReturnCode_t Writer<T>::publish(
        T& data)
{
    auto datawriter_locked = datawriter_.lock_with_exception();

    logDebug(
        AMLIPCPP_DDS_WRITER,
        "Writing message in topic " << topic_ << " from: " << datawriter_locked->guid() << ".");

    return datawriter_locked->write(&data);
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Writer<T>::default_datawriter_qos()
{
    return utils::default_datawriter_qos();
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_IMPL_WRITER_IPP */
