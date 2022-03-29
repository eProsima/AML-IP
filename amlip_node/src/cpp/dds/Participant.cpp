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

#include <ddsrouter_utils/exception/InitializationException.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

const DomainIdType Participant::DEFAULT_DOMAIN_ID_(100u);

using namespace eprosima::fastdds::dds;

Participant::Participant(
        types::AmlipId id,
        const eprosima::fastdds::dds::DomainParticipantQos& qos /* = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT */,
        const DomainIdType& domain /* = DEFAULT_DOMAIN_ID_ */)
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
            participant->delete_contained_entities();
            DomainParticipantFactory::get_instance()->delete_participant(participant);
        }
    );
    if (nullptr == participant_)
    {
        throw ddsrouter::utils::InitializationException("Failed to create participant.");
    }

    // TODO: create subscriber_/publisher_ in create_reader/writer when supporting partitions
    publisher_.reset(
        participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr),
        [this](eprosima::fastdds::dds::Publisher* publisher)
        {
            // deleter for shared ptr
            this->participant_->delete_publisher(publisher);
        }
    );
    if (nullptr == publisher_)
    {
        throw ddsrouter::utils::InitializationException("Failed to create publisher.");
    }

    subscriber_.reset(
        participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr),
        [this](eprosima::fastdds::dds::Subscriber* subscriber)
        {
            // deleter for shared ptr
            this->participant_->delete_subscriber(subscriber);
        }
    );
    if (nullptr == subscriber_)
    {
        throw ddsrouter::utils::InitializationException("Failed to create subscriber.");
    }
}

Participant::~Participant()
{
}

types::AmlipId Participant::id() const noexcept
{
    return id_;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
