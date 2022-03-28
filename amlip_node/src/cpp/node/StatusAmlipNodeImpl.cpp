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
 * @file StatusAmlipNodeImpl.cpp
 */

#include <network/topics.hpp>
#include <node/StatusAmlipNodeImpl.hpp>

namespace eprosima {
namespace amlip {
namespace node {

StatusAmlipNodeImpl::StatusAmlipNodeImpl(
        const std::function<void(types::Status)> callback)
    : AmlipNodeImpl()
    , status_reader_(nullptr)
    , callback_(callback)
    , stop_(true)
{
    // Write current status
    write_status_(types::StatusKind::RUNNING);

    // Create status reader from this participant
    eprosima::fastdds::dds::DataReaderQos qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
    qos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
    status_reader_ = participant_->create_reader<types::Status>(network::STATUS_TOPIC, qos);
}

StatusAmlipNodeImpl::~StatusAmlipNodeImpl()
{
    // Notify end of execution
    write_status_(types::StatusKind::DISABLED);

    // TODO: destroy reader correctly
    status_reader_.reset();
}

void StatusAmlipNodeImpl::spin()
{
    stop_.store(false);
    while (!stop_)
    {
        // Wait for new data
        status_reader_->wait_data_available();

        // Check there is data available or it has been awake for stop (or any other future reason)
        if (!status_reader_->is_data_available())
        {
            continue;
        }

        // New data available, take it and call callback
        types::Status status = status_reader_->read();
        callback_(status);
    }
}

void StatusAmlipNodeImpl::stop()
{
    // Stop this entity
    stop_.store(true);

    // Stop Reader
    status_reader_->stop();
}

types::NodeKind StatusAmlipNodeImpl::node_kind_() const noexcept
{
    return types::NodeKind::STATUS;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
