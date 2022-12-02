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
 * @file MainNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <dds/multiservice/MultiServiceClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/MainNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

MainNode::MainNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::main, types::StateKind::stopped, domain_id)
    , job_client_(participant_->create_multiservice_client<types::JobDataType, types::JobSolutionDataType>(
                dds::utils::JOB_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_MAIN, "Created new Main Node: " << *this << ".");
}

MainNode::MainNode(
        const char* name)
    : MainNode(name, dds::Participant::default_domain_id())
{
}

MainNode::MainNode(
        const std::string& name)
    : MainNode(name.c_str())
{
}

MainNode::~MainNode()
{
    logDebug(AMLIPCPP_NODE_MAIN, "Destroying Main Node: " << *this << ".");
}

types::JobSolutionDataType MainNode::request_job_solution(
        const types::JobDataType& data)
{
    types::AmlipIdDataType _;
    return request_job_solution(data, _);
}

types::JobSolutionDataType MainNode::request_job_solution(
        const types::JobDataType& data,
        types::AmlipIdDataType& id)
{
    change_status_(types::StateKind::running);
    types::JobSolutionDataType solution = job_client_->send_request_sync(data, id);
    change_status_(types::StateKind::stopped);
    return solution;
}

std::ostream& operator <<(
        std::ostream& os,
        const MainNode& node)
{
    os << "MAIN_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
