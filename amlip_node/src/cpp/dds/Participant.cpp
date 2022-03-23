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
    , participant_(DomainParticipantFactory::get_instance()->create_participant(domain, qos))
{
    assert (nullptr != participant_);

    // TODO: create subscriber_/publisher_ in create_reader/writer when supporting partitions
    publisher_ = std::shared_ptr<eprosima::fastdds::dds::Publisher>(
        participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr));
    assert (nullptr != publisher_);

    subscriber_ = std::shared_ptr<eprosima::fastdds::dds::Subscriber>(
        participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr));
    assert (nullptr != subscriber_);
}

Participant::~Participant()
{
    if (nullptr != participant_)
    {
        for (auto topic : topics_)
        {
            participant_->delete_topic(topic.second.get());
        }

        if (publisher_ != nullptr)
        {
            for (auto writer : writers_)
            {
                publisher_->delete_datawriter(writer.get());
            }
            participant_->delete_publisher(publisher_.get());
        }

        if (subscriber_ != nullptr)
        {
            for (auto reader : readers_)
            {
                subscriber_->delete_datareader(reader.get());
            }
            participant_->delete_subscriber(subscriber_.get());
        }

        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_.get());
    }
}

types::AmlipId Participant::id()
{
    return id_;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
