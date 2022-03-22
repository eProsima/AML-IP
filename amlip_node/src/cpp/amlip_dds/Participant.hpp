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
 * @file Participant.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPDDS_PARTICIPANT_HPP
#define AMLIP__SRC_CPP_AMLIPDDS_PARTICIPANT_HPP

namespace eprosima {
namespace amlip {
namespace dds {

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>

#include <amlip_types/AmlipId.hpp>
#include <amlip_dds/Reader.hpp>
#include <amlip_dds/Writer.hpp>
#include <amlip_dds/DirectWriter.hpp>
#include <amlip_dds/MultiServiceClient.hpp>
#include <amlip_dds/MultiServiceServer.hpp>

class Participant
{
public:

    Participant(
        AmlipId id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos);

    virtual ~Participant();

    template<typename T>
    std::shared_ptr<Reader<T>> create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos = Reader::default_datareader_qos());

    template<typename T>
    std::shared_ptr<Writer<T>> create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos = Writer::default_datawriter_qos());

    template<typename T>
    std::shared_ptr<DirectWriter<T>> create_direct_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos = Writer::default_datawriter_qos());

    template<typename Task, typename TaskSolution>
    std::shared_ptr<MultiServiceClient<Task, TaskSolution>> create_multiservice_client(
        const std::string& service_name);

    template<typename Task, typename TaskSolution>
    std::shared_ptr<MultiServiceServer<Task, TaskSolution>> create_multiservice_server(
        const std::string& service_name);
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_PARTICIPANT_HPP */
