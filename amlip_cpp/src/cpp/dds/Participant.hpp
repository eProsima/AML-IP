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

#include <ddsrouter_utils/memory/OwnerPtr.hpp>

#include <amlip_cpp/types/AmlipId.hpp>
#include <dds/DdsHandler.hpp>

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
        types::AmlipId id,
        eprosima::fastdds::dds::DomainParticipantQos qos = Participant::default_participant_qos(),
        DomainIdType domain = Participant::default_domain_id());

    //! Participant destructor
    virtual ~Participant();

    //! Id associated with this Participant
    types::AmlipId id() const noexcept;

    //! Name associated with this Participant Id
    std::string name() const noexcept;

    // TODO: add reader methods

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

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_PARTICIPANT_HPP */
