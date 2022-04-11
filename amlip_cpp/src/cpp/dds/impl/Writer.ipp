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

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_WRITER_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_WRITER_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Writer<T>::Writer(
        const std::string& topic,
        std::unique_ptr<WriterListener> listener,
        std::shared_ptr<eprosima::fastdds::dds::DataWriter> datawriter)
    : topic_(topic)
    , data_writer_(datawriter)
    , listener_(listener)
{
}

template <typename T>
Writer<T>::~Writer()
{
    // Reset listener in case it was waiting for matching
    listener_.reset();
}

template <typename T>
eprosima::fastrtps::types::ReturnCode_t Writer<T>::publish(const T& data)
{
    return data_writer_->write(&data);
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Writer<T>::default_datawriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;

    // Preallocated with realloc
    qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    return qos;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_WRITER_IPP */
