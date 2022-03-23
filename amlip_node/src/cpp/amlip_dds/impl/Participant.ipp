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

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template<typename T>
std::shared_ptr<Reader<T>> Participant::create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    
}

template<typename T>
std::shared_ptr<Writer<T>> Participant::create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{

}

template<typename T>
std::shared_ptr<DirectWriter<T>> Participant::create_direct_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{

}

template<typename Task, typename TaskSolution>
std::shared_ptr<MultiServiceClient<Task, TaskSolution>> Participant::create_multiservice_client(
        const std::string& service_name)
{

}

template<typename Task, typename TaskSolution>
std::shared_ptr<MultiServiceServer<Task, TaskSolution>> Participant::create_multiservice_server(
        const std::string& service_name)
{

}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP */
