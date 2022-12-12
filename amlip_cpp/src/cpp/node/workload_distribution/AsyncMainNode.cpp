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
 * @file AsyncMainNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <dds/multiservice/AsyncMultiServiceClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

struct SolutionListenerCast : public dds::SolutionListener<types::JobSolutionDataType>
{
    SolutionListenerCast(std::shared_ptr<node::SolutionListener> listener)
        : listener_(listener)
    {
    }

    void solution_received (
            std::unique_ptr<types::JobSolutionDataType> solution,
            const types::TaskId& task_id,
            const types::AmlipIdDataType&,
            const types::AmlipIdDataType& server_id) override
    {
        listener_->solution_received(std::move(solution), task_id, server_id);
    }

    std::shared_ptr<node::SolutionListener> listener_;
};

AsyncMainNode::AsyncMainNode(
        const char* name,
        std::shared_ptr<SolutionListener> listener,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::main, types::StateKind::stopped, domain_id)
    , job_client_(participant_->create_async_multiservice_client<types::JobDataType, types::JobSolutionDataType>(
                dds::utils::JOB_TOPIC_NAME))
{
    job_client_->run(std::make_shared<SolutionListenerCast>(listener));
    logInfo(AMLIPCPP_NODE_ASYNCMAIN, "Created new Async Main Node: " << *this << ".");
}

AsyncMainNode::AsyncMainNode(
        const char* name,
        std::shared_ptr<SolutionListener> listener)
    : AsyncMainNode(name, listener, dds::Participant::default_domain_id())
{
}

AsyncMainNode::~AsyncMainNode()
{
    job_client_->stop();
    logDebug(AMLIPCPP_NODE_ASYNCMAIN, "Destroying Async Main Node: " << *this << ".");
}

types::TaskId AsyncMainNode::request_job_solution(
        std::shared_ptr<types::JobDataType> data)
{
    return job_client_->send_request_async(data);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
