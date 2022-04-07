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

#ifndef AMLIPCPP__SRC_CPP_DDS_PARTICIPANT_HPP
#define AMLIPCPP__SRC_CPP_DDS_PARTICIPANT_HPP

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>

#include <amlip_cpp/types/AmlipId.hpp>
#include <types/AmlipGenericTopicDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

// Use FastDDS Domain Id type
using DomainIdType = eprosima::fastdds::dds::DomainId_t;

/**
 * @brief TODO
 *
 */
class Participant
{
public:

    /**
     * @brief Construct a new Participant object
     *
     * This constructor creates a new DDS DomainParticipant from fastdds and one Publisher and one Subscriber.
     * All the entities created will be associated with a shared ptr, and will be automatically deleted when the
     * shared ptr is not referenced.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     * @param qos QoS of the DDS DomainParticipant
     * @param domain DomainId of the DDS DomainParticipant
     *
     * @throw \c InitializationException if the DDS DomainParticipant could not be created
     */
    Participant(
        const types::AmlipId& id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos = Participant::default_participant_qos_(),
        const DomainIdType& domain = Participant::default_domain_id_());

    //! Participant destructor
    virtual ~Participant();

    //! Id associated with this Participant
    types::AmlipId id() const noexcept;

    //! Name associated with this Participant Id
    std::string name() const noexcept;

protected:

    /**
     * @brief Return a default Participant QoS, based QoS for every Participant in amlip
     *
     * This QoS set Memory policy to preallocated with realloc.
     *
     * @return Default \c DomainParticipantQos
     */
    static eprosima::fastdds::dds::DomainParticipantQos default_participant_qos_() noexcept;

    //! Default Publisher QoS for every DDS Publisher in amlip
    static eprosima::fastdds::dds::PublisherQos default_publisher_qos_() noexcept;

    //! Default Subscriber QoS for every DDS Subscriber in amlip
    static eprosima::fastdds::dds::SubscriberQos default_subscriber_qos_() noexcept;

    //! Default Domain Id for every amlip participant
    static DomainIdType default_domain_id_() noexcept;

    /**
     * @brief Register TypeSupport related with \c TopicDataType of \c T if it is not already registered.
     *
     * This TypeSupport is a shared ptr and will be automatically deleted when the shared ptr is not referenced.
     *
     * @tparam T \c TopicDataType associated with the TypeSupport required
     * @return RETCODE_OK if registration was successful
     * @return RETCODE_ERROR if registration failed
     * @return RETCODE_PRECONDITION_NOT_MET if type was already registered
     *
     * @throw \c InitializationException if the topic could not be created
     *
     * @warning Do not use this type support once the Participant is deleted
     */
    template<typename T>
    eprosima::fastrtps::types::ReturnCode_t register_type_() noexcept;

    /**
     * @brief Get the topic object related with type \c T and topic name.
     *
     * If the topic has been already created, take the Topic, if not create a new one.
     * This Topic is a shared ptr and will be automatically deleted when the shared ptr is not referenced.
     *
     * @tparam T type related with the topic
     * @param topic_name name of the topic
     * @return reference to the Topic
     *
     * @throw \c InitializationException if the topic could not be created
     *
     * @warning Do not use this type support once the Participant is deleted
     */
    template<typename T>
    std::shared_ptr<eprosima::fastdds::dds::Topic> get_topic_(const std::string& topic_name);

    /////
    // INTERNAL VARIABLES

    //! Id identifying this Participant
    types::AmlipId id_;

    //! Shared ptr referencing the DDS DomainParticipant
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;

    //! Shared ptr referencing the DDS Publisher
    std::shared_ptr<eprosima::fastdds::dds::Publisher> publisher_;

    //! Shared ptr referencing the DDS Subscriber
    std::shared_ptr<eprosima::fastdds::dds::Subscriber> subscriber_;

    /**
     * @brief Map with references to the Topics already created from this Participant
     *
     * Every Time a Topic is get from this Participant, it stores a share reference of it.
     * If the Topic is get again, it will return a reference to this same Topic.
     *
     * Topic is indexed by a duple of topic name and topic type
     */
    std::map<std::pair<std::string, std::string>, std::shared_ptr<eprosima::fastdds::dds::Topic>> topics_;

    /**
     * @brief Map with references to the TypeSupports already created from this Participant
     *
     * Every registered type is stored in this map.
     * TypeSupport index is the name of the DataType of the internal \c AmlipGenericTopicDataType
     *
     * @note this may not me needed, as TypeSupport is not required after being created.
     */
    std::map<std::string, eprosima::fastdds::dds::TypeSupport> types_;

    /////
    // STATIC CONST VARIABLES

    //! Default DomainId for every DomainParticipant in amlip
    static const DomainIdType DEFAULT_DOMAIN_ID_; // 166

};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Participant.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_PARTICIPANT_HPP */
