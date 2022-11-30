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
 * @file ParentNode.cpp
 */

#include <cpp_utils/Log.hpp>

#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/ParentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ParentNode::ParentNode(
        const char* name,
        types::NodeKind node_kind,
        types::StateKind initial_state)
    : participant_(std::make_unique<dds::Participant>(name))
    , status_writer_(participant_->create_writer<types::StatusDataType>(
                dds::utils::STATUS_TOPIC_NAME,
                dds::utils::status_writer_qos()))
    , current_state_(initial_state)
    , node_kind_(node_kind)
{
    logDebug(AMLIPCPP_NODE_STATUS, "Created new Node: " << *this << ".");
    publish_status_();
}

ParentNode::ParentNode(
        const char* name,
        types::NodeKind node_kind)
    : ParentNode(name, node_kind, types::StateKind::stopped)
{
}

ParentNode::ParentNode(
        const std::string& name,
        types::NodeKind node_kind)
    : ParentNode(name.c_str(), node_kind)
{
}

ParentNode::~ParentNode()
{
    logDebug(AMLIPCPP_NODE_STATUS, "Destroying Node: " << *this << ".");

    change_status_(types::StateKind::dropped);

    logDebug(AMLIPCPP_NODE_STATUS, "Node destroyed.");
}

types::AmlipIdDataType ParentNode::id() const noexcept
{
    return participant_->id();
}

types::StateKind ParentNode::current_state() const noexcept
{
    return current_state_;
}

types::NodeKind ParentNode::node_kind() const noexcept
{
    return node_kind_;
}

void ParentNode::change_status_(
        const types::StateKind& new_state) noexcept
{
    current_state_ = new_state;
    publish_status_();
}

void ParentNode::publish_status_() const noexcept
{
    // Create status data
    types::StatusDataType status(
        id(),
        node_kind(),
        current_state());

    // Publish status
    status_writer_->publish(status);
}

std::ostream& operator <<(
        std::ostream& os,
        const ParentNode& node)
{
    os << "NODE{" << node.id() << ";" << node.current_state() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
