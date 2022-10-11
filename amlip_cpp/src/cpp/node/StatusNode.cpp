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
 * @file StatusNode.cpp
 */

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <network/topic.hpp>
#include <node/StatusNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

StatusNode::StatusNode(const char* name)
    : ParentNode(name, types::NodeKind::STATUS)
    , status_reader_(participant_.create_reader<types::StatusDataType>(
        network::STATUS_TOPIC_NAME,
        network::status_reader_qos()))
    , processing_(false)
{
    logInfo(AMLIPCPP_NODE_STATUS, "Created new Status Node: " << *this << ".");
}

StatusNode::StatusNode(const std::string& name)
    : StatusNode(name.c_str())
{
}

StatusNode::~StatusNode()
{
    logDebug(AMLIPCPP_NODE_STATUS, "Destroying Status Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (processing_)
    {
        stop_processing();
    }

    logDebug(AMLIPCPP_NODE_STATUS, "Status Node Destroyed.");
}

void StatusNode::process_status_async(
        const std::function<void(const types::StatusDataType&)>& callback)
{
    if (processing_)
    {
        throw ddsrouter::utils::InconsistencyException(
            STR_ENTRY << "Status node " << this << " is already processing data.");
    }
    else
    {
        processing_ = true;
        process_thread_ = std::thread(&StatusNode::process_routine_, this, callback);

        change_status_(types::StateKind::RUNNING);
    }
}

void StatusNode::stop_processing()
{
    if (processing_)
    {
        processing_ = false;
        status_reader_->awake_waiting_threads(); // This must awake thread and it must finish
        process_thread_.join();

        change_status_(types::StateKind::STOPPED);
    }
}

void StatusNode::process_routine_(
        const std::function<void(const types::StatusDataType&)>& callback)
{
    while(processing_)
    {
        // Wait for data
        ddsrouter::event::AwakeReason reason = status_reader_->wait_data_available();

        if (reason == ddsrouter::event::AwakeReason::DISABLED)
        {
            logDebug(AMLIPCPP_NODE_STATUS, "Status Node " << *this << " finished processing data.");

            // Break thread execution
            return;
        }

        // Read data
        types::StatusDataType status = status_reader_->read();

        logDebug(AMLIPCPP_NODE_STATUS, "Status Node " << *this << " read data :" << status << ".");

        // Call callback
        callback(status);
    }
}

std::ostream& operator <<(
        std::ostream& os,
        const StatusNode& node)
{
    os << "STATUS_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
