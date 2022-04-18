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

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

const DomainIdType Participant::DEFAULT_DOMAIN_ID_(166u);

using namespace eprosima::fastdds::dds;

Participant::Participant(
        types::AmlipId id,
        eprosima::fastdds::dds::DomainParticipantQos qos /* = Participant::default_participant_qos() */,
        DomainIdType domain /* = Participant::default_domain_id() */)
    : id_(id)
{
    // Set Participant name
    qos.name(id.name());

    // Create DDS Handler
    dds_handler_.reset(
        new DdsHandler(qos, domain));
}

Participant::~Participant()
{
    // Destroy DdsHandler that will destroy every DDS subentity
    dds_handler_.reset();
}

types::AmlipId Participant::id() const noexcept
{
    return id_;
}

std::string Participant::name() const noexcept
{
    return id_.name();
}

eprosima::fastdds::dds::DomainParticipantQos Participant::default_participant_qos() noexcept
{
    return eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
}

DomainIdType Participant::default_domain_id() noexcept
{
    return DEFAULT_DOMAIN_ID_;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
