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

// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

class Participant
{
public:

    Participant(
        AmlipId id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos,
        const DomainIdType& domain=DEFAULT_DOMAIN_ID_);

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

    AmlipId id();

protected:

    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::Reader> create_reader_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos = Reader::default_datareader_qos());

    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::Writer> create_writer_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos = Writer::default_datawriter_qos());

    types::AmlipId id_;

    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;

    // TODO: support multiple partitions
    // std::map<std::string, std::shared_ptr<eprosima::fastdds::dds::Publisher>> publishers_;
    std::shared_ptr<eprosima::fastdds::dds::Publisher> publisher_;

    // TODO: support multiple partitions
    // std::map<std::string, std::shared_ptr<eprosima::fastdds::dds::Subscriber>> subscribers_;
    std::shared_ptr<eprosima::fastdds::dds::Subscriber> subscriber_;

    std::vector<std::shared_ptr<eprosima::fastdds::dds::DataWriter>> writers_;

    std::vector<std::shared_ptr<eprosima::fastdds::dds::DataReader>> readers_;

    std::map<std::tuple<std::string, std::string>, eprosima::fastdds::dds::Topic> topics_;

    std::map<std::string, std::shared_ptr<types::IBaseAmlipGenericTopicDataType>> types_;

    static const DomainIdType DEFAULT_DOMAIN_ID_; // 100

};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <impl/Participant.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPDDS_PARTICIPANT_HPP */
