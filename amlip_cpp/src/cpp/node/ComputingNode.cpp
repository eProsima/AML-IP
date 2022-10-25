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
 * @file ComputingNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <network/topic.hpp>
#include <node/ComputingNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ComputingNode::ComputingNode(const char* name)
    : ParentNode(name, types::NodeKind::COMPUTING)
    , job_server_(participant_.create_multiservice_server<types::JobDataType, types::SolutionDataType>(
        network::JOB_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_COMPUTING, "Created new Computing Node: " << *this << ".");
}

ComputingNode::ComputingNode(const std::string& name)
    : ComputingNode(name.c_str())
{
}

ComputingNode::~ComputingNode()
{
    logDebug(AMLIPCPP_NODE_COMPUTING, "Destroying Computing Node: " << *this << ".");
}

types::MsReferenceDataType ComputingNode::process_job(
        const std::function<types::SolutionDataType(const types::JobDataType&)>& callback)
{
    return job_server_->process_task_sync(callback);
}

std::ostream& operator <<(
        std::ostream& os,
        const ComputingNode& node)
{
    os << "COMPUTING_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
