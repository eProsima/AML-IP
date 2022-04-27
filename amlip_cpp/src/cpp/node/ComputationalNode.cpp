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
 * @file ComputationalNode.cpp
 */

#include <ddsrouter_utils/Log.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <network/topic.hpp>
#include <node/ComputationalNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ComputationalNode::ComputationalNode(const char* name)
    : ParentNode(name, types::NodeKind::COMPUTATIONAL)
    , job_server_(participant_.create_multiservice_server<types::JobDataType, types::SolutionDataType>(
        network::JOB_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_COMPUTATIONAL, "Created new Computational Node: " << *this << ".");
}

ComputationalNode::ComputationalNode(const std::string& name)
    : ComputationalNode(name.c_str())
{
}

ComputationalNode::~ComputationalNode()
{
    logDebug(AMLIPCPP_NODE_COMPUTATIONAL, "Destroying Computational Node: " << *this << ".");
}

types::MsReferenceDataType ComputationalNode::process_job(
        const std::function<types::SolutionDataType(const types::JobDataType&)>& callback)
{
    return job_server_->process_task_sync(callback);
}

std::ostream& operator <<(
        std::ostream& os,
        const ComputationalNode& node)
{
    os << "COMPUTATIONAL_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
