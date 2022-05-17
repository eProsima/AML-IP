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
 * @file Participant.cpp
 */

#include <ddsrouter_utils/Log.hpp>

#include <dds/Participant.hpp>
#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

const DomainIdType Participant::DEFAULT_DOMAIN_ID_(166u);

using namespace eprosima::fastdds::dds;

Participant::Participant(
        types::AmlipIdDataType id,
        eprosima::fastdds::dds::DomainParticipantQos qos /* = Participant::default_participant_qos() */,
        DomainIdType domain /* = Participant::default_domain_id() */)
    : id_(id)
{
    logDebug(AMLIPCPP_PARTICIPANT, "Creating Participant with id " << id << " in domain " << domain << ".");

    // Set Participant name
    qos.name(id.name());

    // Create DDS Handler
    dds_handler_.reset(
        new DdsHandler(qos, domain));

    logDebug(AMLIPCPP_PARTICIPANT, "Participant " << *this << " created.");
}

Participant::Participant(
        const std::string& name,
        eprosima::fastdds::dds::DomainParticipantQos qos /* = Participant::default_participant_qos() */,
        DomainIdType domain /* = Participant::default_domain_id() */)
    : Participant(types::AmlipIdDataType(name), qos, domain)
{
}

Participant::Participant(
        const char* name,
        eprosima::fastdds::dds::DomainParticipantQos qos /* = Participant::default_participant_qos() */,
        DomainIdType domain /* = Participant::default_domain_id() */)
    : Participant(types::AmlipIdDataType(name), qos, domain)
{
}

Participant::~Participant()
{
    logDebug(AMLIPCPP_PARTICIPANT, "Destroying Participant " << *this << ".");

    // Destroy DdsHandler that will destroy every DDS subentity
    dds_handler_.reset();

    logDebug(AMLIPCPP_PARTICIPANT, "Participant " << *this << " destroyed.");
}

types::AmlipIdDataType Participant::id() const noexcept
{
    return id_;
}

std::string Participant::name() const noexcept
{
    return id_.name();
}

eprosima::fastdds::dds::DomainParticipantQos Participant::default_participant_qos() noexcept
{
    return utils::default_domain_participant_qos();
}

DomainIdType Participant::default_domain_id() noexcept
{
    return DEFAULT_DOMAIN_ID_;
}

std::ostream& operator <<(
        std::ostream& os,
        const Participant& participant)
{
    os << "PARTICIPANT{" << participant.id() << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
