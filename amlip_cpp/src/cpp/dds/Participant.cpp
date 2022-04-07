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

#include <assert.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <ddsrouter_utils/exception/InconsistencyException.hpp>
#include <ddsrouter_utils/exception/InitializationException.hpp>

#include <dds/Participant.hpp>
#include <types/AmlipGenericTopicDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

const DomainIdType Participant::DEFAULT_DOMAIN_ID_(166u);

using namespace eprosima::fastdds::dds;

Participant::Participant(
        const types::AmlipId& id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos /* = Participant::default_participant_qos_() */,
        const DomainIdType& domain /* = Participant::default_domain_id_() */)
    : id_(id)
    , participant_(nullptr)
    , publisher_(nullptr)
    , subscriber_(nullptr)
{
    participant_.reset(
        DomainParticipantFactory::get_instance()->create_participant(domain, qos),
        [](eprosima::fastdds::dds::DomainParticipant* participant)
        {
            // deleter for shared ptr
            if (!participant->delete_contained_entities())
            {
                logError(AMLIP_DDS_PARTICIPANT, "Fail deleting contained entities.");
            }

            if (!DomainParticipantFactory::get_instance()->delete_participant(participant))
            {
                logError(AMLIP_DDS_PARTICIPANT, "Fail deleting participant.");
            }
        }
    );
    if (nullptr == participant_)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create participant " << id.name() << ".");
    }

    publisher_.reset(
        participant_->create_publisher(default_publisher_qos_(), nullptr),
        [this](eprosima::fastdds::dds::Publisher* publisher)
        {
            // deleter for shared ptr
            if (!this->participant_->delete_publisher(publisher))
            {
                logError(AMLIP_DDS_PARTICIPANT, "Fail deleting publisher.");
            }
        }
    );
    if (nullptr == publisher_)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create publisher in participant " << id.name() << ".");
    }

    subscriber_.reset(
        participant_->create_subscriber(default_subscriber_qos_(), nullptr),
        [this](eprosima::fastdds::dds::Subscriber* subscriber)
        {
            // deleter for shared ptr
            if(!this->participant_->delete_subscriber(subscriber))
            {
                logError(AMLIP_DDS_PARTICIPANT, "Fail deleting subscriber.");
            }
        }
    );
    if (nullptr == subscriber_)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create subscriber in participant " << id.name() << ".");
    }
}

Participant::~Participant()
{
    // Destroy subentities before participant
    topics_.clear();
    types_.clear();
    publisher_.reset();
    subscriber_.reset();

    // Destroy Participant
    participant_.reset();
}

types::AmlipId Participant::id() const noexcept
{
    return id_;
}

std::string Participant::name() const noexcept
{
    return id_.name();
}

eprosima::fastdds::dds::DomainParticipantQos Participant::default_participant_qos_() noexcept
{
    return eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
}

DomainIdType Participant::default_domain_id_() noexcept
{
    return DEFAULT_DOMAIN_ID_;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
