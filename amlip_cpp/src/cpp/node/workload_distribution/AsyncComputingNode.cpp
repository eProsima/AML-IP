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
 * @file AsyncComputingNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/node/workload_distribution/AsyncComputingNode.hpp>

#include <dds/multiservice/AsyncMultiServiceServer.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

struct TaskListenerCast : public dds::TaskListener<types::JobDataType, types::JobSolutionDataType>
{
    TaskListenerCast(std::shared_ptr<JobListener> listener)
        : listener_(listener)
    {
    }

    types::JobSolutionDataType process_task (
            std::unique_ptr<types::JobDataType> task,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id,
            const types::AmlipIdDataType&) override
    {
        return listener_->process_job(std::move(task), task_id, client_id);
    }

    std::shared_ptr<JobListener> listener_;
};

AsyncComputingNode::AsyncComputingNode(
        const char* name,
        std::shared_ptr<JobListener> listener,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::computing, types::StateKind::stopped, domain_id)
    , job_server_(
        participant_->create_async_multiservice_server<types::JobDataType,
        types::JobSolutionDataType>(dds::utils::JOB_TOPIC_NAME))
    , listener_(listener)
{
    logInfo(AMLIPCPP_NODE_ASYNCCOMPUTING, "Created new Async Computing Node: " << *this << ".");
}

AsyncComputingNode::AsyncComputingNode(
        const char* name,
        std::shared_ptr<JobListener> listener)
    : AsyncComputingNode(name, listener, dds::Participant::default_domain_id())
{
    // Do nothing
}

AsyncComputingNode::~AsyncComputingNode()
{
    stop();
    logDebug(AMLIPCPP_NODE_ASYNCCOMPUTING, "Destroying Async Computing Node: " << *this << ".");
}

void AsyncComputingNode::run()
{
    job_server_->run(std::make_shared<TaskListenerCast>(listener_));
}

void AsyncComputingNode::stop()
{
    job_server_->stop();
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
