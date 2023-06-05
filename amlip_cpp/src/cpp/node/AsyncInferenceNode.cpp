// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file AsyncInferenceNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/node/AsyncInferenceNode.hpp>

#include <dds/multiservice/AsyncMultiServiceServer.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

struct TaskListenerCast : public dds::TaskListener<types::InferenceDataType, types::InferenceSolutionDataType>
{
    TaskListenerCast(
            const std::shared_ptr<InferenceReplier>& listener)
        : listener_(listener)
    {
    }

    types::InferenceSolutionDataType process_task (
            std::unique_ptr<types::InferenceDataType>&& task,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id,
            const types::AmlipIdDataType&) override
    {
        return listener_->process_inference(*task, task_id, client_id);
    }

    std::shared_ptr<InferenceReplier> listener_;
};

AsyncInferenceNode::AsyncInferenceNode(
        const char* name,
        const std::shared_ptr<InferenceReplier>& listener,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::inference, types::StateKind::stopped, domain_id)
    , inference_server_(participant_->create_async_multiservice_server<types::InferenceDataType,
            types::InferenceSolutionDataType>(
                dds::utils::INFERENCE_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_ASYNCINFERENCE, "Created new Inference Node: " << *this << ".");
}

AsyncInferenceNode::AsyncInferenceNode(
        const char* name,
        const std::shared_ptr<InferenceReplier>& listener)
    : AsyncInferenceNode(name, listener, dds::Participant::default_domain_id())
{
    // Do nothing
}

AsyncInferenceNode::AsyncInferenceNode(
        const std::string& name,
        const std::shared_ptr<InferenceReplier>& listener)
    : AsyncInferenceNode(name.c_str(), listener, dds::Participant::default_domain_id())
{
    // Do nothing
}

AsyncInferenceNode::~AsyncInferenceNode()
{
    stop();
    logDebug(AMLIPCPP_NODE_ASYNCINFERENCE, "Destroying Inference Node: " << *this << ".");
}

void AsyncInferenceNode::run()
{
    inference_server_->run(std::make_shared<TaskListenerCast>(listener_));
}

void AsyncInferenceNode::stop()
{
    inference_server_->stop();
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
