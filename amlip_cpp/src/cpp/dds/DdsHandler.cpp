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
 * @file DdsHandler.cpp
 */

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <ddsrouter_utils/Log.hpp>

#include <dds/DdsHandler.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

using namespace eprosima::fastdds::dds;

DdsHandler::DdsHandler(
        const eprosima::fastdds::dds::DomainParticipantQos& qos,
        const DomainIdType& domain)
{
    logDebug(AMLIPCPP_DDSHANDLER, "Creating DdsHandler in domain " << domain << ".");

    // CREATE FASTDDS DOMAIN PARTICIPANT
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
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create participant " << qos.name() << ".");
    }

    // CREATE FASTDDS PUBLISHER
    publisher_.reset(
        participant_->create_publisher(default_publisher_qos_(), nullptr),
        [this](eprosima::fastdds::dds::Publisher* publisher)
        {
            // deleter for shared ptr
            this->participant_->delete_publisher(publisher);
        }
    );
    if (nullptr == publisher_)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create publisher in participant " << qos.name() << ".");
    }

    // CREATE FASTDDS SUBSCRIBER
    subscriber_.reset(
        participant_->create_subscriber(default_subscriber_qos_(), nullptr),
        [this](eprosima::fastdds::dds::Subscriber* subscriber)
        {
            // deleter for shared ptr
            this->participant_->delete_subscriber(subscriber);
        }
    );
    if (nullptr == subscriber_)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create subscriber in participant " << qos.name() << ".");
    }

    logDebug(AMLIPCPP_DDSHANDLER, "DdsHandler created with GUID: " << participant_->guid() << ".");
}

DdsHandler::~DdsHandler()
{
    logDebug(AMLIPCPP_DDSHANDLER, "Destroying DdsHandler.");

    // Destroy object in correct order
    // Each OwnerPtr has already the deleter for each internal value
    datawriters_.clear();
    datareaders_.clear();
    topics_.clear();
    publisher_.reset();
    subscriber_.reset();
    participant_.reset();

    logDebug(AMLIPCPP_DDSHANDLER, "DdsHandler destroyed.");
}

eprosima::fastdds::dds::PublisherQos DdsHandler::default_publisher_qos_() noexcept
{
    return eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
}

eprosima::fastdds::dds::SubscriberQos DdsHandler::default_subscriber_qos_() noexcept
{
    return eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
