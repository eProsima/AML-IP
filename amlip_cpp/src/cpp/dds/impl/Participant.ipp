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
 * @file Participant.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_PARTICIPANT_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_PARTICIPANT_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
std::shared_ptr<Writer<T>> Participant::create_writer(
        const std::string& topic_name,
        eprosima::fastdds::dds::DataWriterQos qos /* = Writer<T>::default_datawriter_qos() */)
{
    return std::make_shared<Writer<T>>(
        topic_name,
        dds_handler_.lease(),
        qos);
}

template <typename T>
std::shared_ptr<Reader<T>> Participant::create_reader(
        const std::string& topic_name,
        eprosima::fastdds::dds::DataReaderQos qos /* = Reader<T>::default_datareader_qos() */)
{
    return std::make_shared<Reader<T>>(
        topic_name,
        dds_handler_.lease(),
        qos);
}

template <typename T>
std::shared_ptr<DirectWriter<T>> Participant::create_direct_writer(
        const std::string& topic_name,
        eprosima::fastdds::dds::DataWriterQos qos /* = DirectWriter<T>::default_directwriter_qos() */)
{
    return std::make_shared<DirectWriter<T>>(
        topic_name,
        dds_handler_.lease(),
        qos);
}

template <typename T>
std::shared_ptr<TargetedReader<T>> Participant::create_targeted_reader(
        const std::string& topic_name,
        eprosima::fastdds::dds::DataReaderQos qos /* = TargetedReader<T>::default_targetedreader_qos() */)
{
    return std::make_shared<TargetedReader<T>>(
        id_,
        topic_name,
        dds_handler_.lease(),
        qos);
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_PARTICIPANT_IPP */
