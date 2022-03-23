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

#include <amlip_network/topics.hpp>
#include <amlip_node/StatusAmlipNodeImpl.hpp>

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
    // Create status writer from this participant
    status_reader_ = participant_->create_reader<types::Status>(network::STATUS_TOPIC);
}

StatusAmlipNodeImpl::~StatusAmlipNodeImpl()
{
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
        Status status = status_reader_->read();
        callback_(status);
    }
}

void StatusAmlipNodeImpl::stop()
{
    // Stop this entity
    stop_.store(true);

    // Stop Reader
    status_reader->stop();
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
