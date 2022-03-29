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

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <amlip_node/types/AmlipId.hpp>
#include <types/AmlipGenericTopicDataType.hpp>
#include <dds/Reader.hpp>
#include <dds/Writer.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

class Participant
{
public:

    Participant(
        types::AmlipId id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT,
        const DomainIdType& domain=DEFAULT_DOMAIN_ID_);

    virtual ~Participant();

    template<typename T>
    std::shared_ptr<Reader<T>> create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos = Reader<T>::default_datareader_qos());

    template<typename T>
    std::shared_ptr<Writer<T>> create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos = Writer<T>::default_datawriter_qos());

    // template<typename T>
    // std::shared_ptr<DirectWriter<T>> create_direct_writer(
    //     const std::string& topic_name,
    //     const eprosima::fastdds::dds::DataWriterQos& qos = Writer::default_datawriter_qos());

    // template<typename Task, typename TaskSolution>
    // std::shared_ptr<MultiServiceClient<Task, TaskSolution>> create_multiservice_client(
    //     const std::string& service_name);

    // template<typename Task, typename TaskSolution>
    // std::shared_ptr<MultiServiceServer<Task, TaskSolution>> create_multiservice_server(
    //     const std::string& service_name);

    types::AmlipId id() const noexcept;

protected:

    template<typename T>
    eprosima::fastdds::dds::TypeSupport register_type() noexcept;

    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::Topic> register_topic(const std::string& topic_name);

    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::DataReader> create_datareader_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos = Reader<T>::default_datareader_qos());

    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::DataWriter> create_datawriter_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos = Writer<T>::default_datawriter_qos());

    types::AmlipId id_;

    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;

    // TODO: support multiple partitions
    // std::map<std::string, eprosimastd::shared_ptr<::fastdds::dds::Publisher>> publishers_;
    std::shared_ptr<eprosima::fastdds::dds::Publisher> publisher_;

    // TODO: support multiple partitions
    // std::map<std::string, eprosimastd::shared_ptr<::fastdds::dds::Subscriber>> subscribers_;
    std::shared_ptr<eprosima::fastdds::dds::Subscriber> subscriber_;

    std::vector<std::shared_ptr<eprosima::fastdds::dds::DataWriter>> datawriters_;

    std::vector<std::shared_ptr<eprosima::fastdds::dds::DataReader>> datareaders_;

    std::map<std::pair<std::string, std::string>, std::shared_ptr<eprosima::fastdds::dds::Topic>> topics_;

    std::map<std::string, eprosima::fastdds::dds::TypeSupport> types_;

    static const DomainIdType DEFAULT_DOMAIN_ID_; // 100

};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Participant.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPDDS_PARTICIPANT_HPP */
