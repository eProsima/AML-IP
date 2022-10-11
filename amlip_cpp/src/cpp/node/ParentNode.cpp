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

#include <ddsrouter_utils/Log.hpp>

#include <network/topic.hpp>
#include <node/ParentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ParentNode::ParentNode(const char* name)
    : participant_(name)
    , current_state_(types::StateKind::STOPPED)
    , status_writer_(participant_.create_writer<types::StatusDataType>(
        network::STATUS_TOPIC_NAME,
        network::status_writer_qos()))
{
}

ParentNode::ParentNode(std::string name)
    : ParentNode(name.c_str())
{
}

ParentNode::~ParentNode()
{
    // Children must publish as dropped in their destructors
}

types::AmlipIdDataType ParentNode::id() const noexcept
{
    return participant_.id();
}

types::StateKind ParentNode::current_state() const noexcept
{
    return current_state_;
}

types::NodeKind ParentNode::node_kind() const noexcept
{
    return types::NodeKind::UNDETERMINED;
}

ddsrouter::utils::ReturnCode ParentNode::run()
{
    // If it is already running, do nothing
    if (current_state_ == types::StateKind::RUNNING)
    {
        return ddsrouter::utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
    else
    {
        current_state_ = types::StateKind::RUNNING;
        ddsrouter::utils::ReturnCode code = run_(); // Children specific run methods

        if (code())
        {
            publish_status_();
        }

        return code;
    }
}

ddsrouter::utils::ReturnCode ParentNode::stop()
{
    // If it is already running, do nothing
    if (current_state_ == types::StateKind::STOPPED)
    {
        return ddsrouter::utils::ReturnCode::RETCODE_PRECONDITION_NOT_MET;
    }
    else
    {
        current_state_ = types::StateKind::STOPPED;
        ddsrouter::utils::ReturnCode code = stop_(); // Children specific run methods

        if (code())
        {
            publish_status_();
        }

        return code;
    }
}

ddsrouter::utils::ReturnCode ParentNode::run_()
{
    // Do nothing
    // Implement in children node if needed
    return ddsrouter::utils::ReturnCode::RETCODE_OK;
}

ddsrouter::utils::ReturnCode ParentNode::stop_()
{
    // Do nothing
    // Implement in children node if needed
    return ddsrouter::utils::ReturnCode::RETCODE_OK;
}

void ParentNode::publish_status_() const
{
    // Create status data
    types::StatusDataType status(
            id(),
            node_kind(),
            current_state());

    // Publish status
    status_writer_->publish(status);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
