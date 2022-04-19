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

#include <ddsrouter_utils/memory/owner_ptr.hpp>

#include <amlip_cpp/types/AmlipId.hpp>
#include <dds/DdsHandler.hpp>
#include <dds/DirectWriter.hpp>
#include <dds/Reader.hpp>
#include <dds/TargetedReader.hpp>
#include <dds/Writer.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

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
     * This constructor creates a new \c DdsHandler that handles all the dds entities.
     * This object owns the \c DdsHandler , that could be leased to subentities.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     * @param qos QoS of the DDS DomainParticipant
     * @param domain DomainId of the DDS DomainParticipant
     *
     * @throw \c InitializationException if the DDS DomainParticipant could not be created
     */
    Participant(
            types::AmlipId id,
            eprosima::fastdds::dds::DomainParticipantQos qos = Participant::default_participant_qos(),
            DomainIdType domain = Participant::default_domain_id());

    Participant(
            const std::string& name,
            eprosima::fastdds::dds::DomainParticipantQos qos = Participant::default_participant_qos(),
            DomainIdType domain = Participant::default_domain_id());

    //! Participant destructor
    virtual ~Participant();

    //! Id associated with this Participant
    types::AmlipId id() const noexcept;

    //! Name associated with this Participant Id
    std::string name() const noexcept;

    template <typename T>
    std::shared_ptr<Writer<T>> create_writer(
            const std::string& topic_name,
            eprosima::fastdds::dds::DataWriterQos qos = Writer<T>::default_datawriter_qos());

    template <typename T>
    std::shared_ptr<Reader<T>> create_reader(
            const std::string& topic_name,
            eprosima::fastdds::dds::DataReaderQos qos = Reader<T>::default_datareader_qos());

    template <typename T>
    std::shared_ptr<DirectWriter<T>> create_direct_writer(
        const std::string& topic_name);

    template <typename T>
    std::shared_ptr<TargetedReader<T>> create_targeted_reader(
        const std::string& topic_name);

    /**
     * @brief Return a default Participant QoS, based QoS for every Participant in amlip
     *
     * Default Participant QoS is:
     * - Entity Factory: disabled
     *
     * @return Default \c DomainParticipantQos
     */
    static eprosima::fastdds::dds::DomainParticipantQos default_participant_qos() noexcept;

    //! Default Domain Id for every amlip participant
    static DomainIdType default_domain_id() noexcept;

protected:

    //! Id identifying this Participant
    const types::AmlipId id_;

    ddsrouter::utils::OwnerPtr<DdsHandler> dds_handler_;

    /////
    // STATIC CONST VARIABLES

    //! Default DomainId for every DomainParticipant in amlip
    static const DomainIdType DEFAULT_DOMAIN_ID_; // 166
};

//! \c Participant to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const Participant& participant);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Participant.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_PARTICIPANT_HPP */
