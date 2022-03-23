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
 * @file AmlipNodeImpl.cpp
 */

#include <network/topics.hpp>
#include <node/AmlipNodeImpl.hpp>

namespace eprosima {
namespace amlip {
namespace node {

AmlipNodeImpl::AmlipNodeImpl()
        : participant_(nullptr)
        , status_writer_(nullptr)
        , id_(types::AmlipId::new_unique_id())
{
    // Create Participant
    participant_ = std::make_shared<dds::Participant>(id_);

    // Create status writer from this participant
    status_writer_ = participant_->create_writer<types::Status>(network::STATUS_TOPIC);

    // Write actual status
    write_status_(types::StatusKind::RUNNING);
}

AmlipNodeImpl::~AmlipNodeImpl()
{
    // TODO: destroy writer correctly
    status_writer_.reset();

    // Destroy Participant
    participant_.reset();
}

void AmlipNodeImpl::write_status_(types::StatusKind status)
{
    // Create data
    types::Status status_data;
    status_data.id(id());
    status_data.node_kind(node_kind_());
    status_data.status(status);

    // Send data
    status_writer_->write(status_data);
}

types::NodeKind AmlipNodeImpl::node_kind_() const noexcept
{
    return types::NodeKind::UNDETERMINED;
}

types::AmlipId AmlipNodeImpl::id() const noexcept
{
    return id_;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
