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

#include <amlip_cpp/node/workload_distribution/ComputingNode.hpp>

#include <dds/multiservice/MultiServiceServer.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ComputingNode::ComputingNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::computing, types::StateKind::stopped, domain_id)
    , job_server_(
        participant_->create_multiservice_server<types::JobDataType,
        types::JobSolutionDataType>(dds::utils::JOB_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_COMPUTING, "Created new Computing Node: " << *this << ".");
}

ComputingNode::ComputingNode(
        const char* name)
    : ComputingNode(name, dds::Participant::default_domain_id())
{
}

ComputingNode::ComputingNode(
        const std::string& name)
    : ComputingNode(name.c_str())
{
}

ComputingNode::~ComputingNode()
{
    logDebug(AMLIPCPP_NODE_COMPUTING, "Destroying Computing Node: " << *this << ".");
}

void ComputingNode::process_job(
        const std::function<types::JobSolutionDataType(const types::JobDataType&)>& callback)
{
    types::AmlipIdDataType _;
    process_job(callback, _);
}

void ComputingNode::process_job(
        const std::function<types::JobSolutionDataType(const types::JobDataType&)>& callback,
        types::AmlipIdDataType& client_id)
{
    change_status_(types::StateKind::running);
    auto reference = job_server_->process_task_sync(callback);
    client_id = reference.client_id();
    change_status_(types::StateKind::stopped);
}

void ComputingNode::process_job(
        const JobListener& listener)
{
    process_job(
        [&listener]
            (const types::JobDataType& job)
        {
            return listener.process_job(job);
        });
}

void ComputingNode::process_job(
        const JobListener& listener,
        types::AmlipIdDataType& client_id)
{
    process_job(
        [&listener]
            (const types::JobDataType& job)
        {
            return listener.process_job(job);
        },
        client_id);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
