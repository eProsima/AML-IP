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
 * @file AsyncEdgeNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <dds/multiservice/AsyncMultiServiceClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/AsyncEdgeNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

struct SolutionListenerCast : public dds::SolutionListener<types::InferenceSolutionDataType>
{
    SolutionListenerCast(
            const std::shared_ptr<node::InferenceSolutionListener>& listener)
        : listener_(listener)
    {
    }

    void solution_received(
            std::unique_ptr<types::InferenceSolutionDataType>&& inference,
            const types::TaskId& task_id,
            const types::AmlipIdDataType&,
            const types::AmlipIdDataType& server_id) override
    {
        listener_->inference_received(*inference, task_id, server_id);
    }

    std::shared_ptr<node::InferenceSolutionListener> listener_;
};

AsyncEdgeNode::AsyncEdgeNode(
        const char* name,
        const std::shared_ptr<InferenceSolutionListener>& listener,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::edge, types::StateKind::running, domain_id)
    , inference_client_(participant_->create_async_multiservice_client<types::InferenceDataType,
            types::InferenceSolutionDataType>(
                dds::utils::INFERENCE_TOPIC_NAME))
{
    inference_client_->run(std::make_shared<SolutionListenerCast>(listener));
    logInfo(AMLIPCPP_NODE_ASYNCMAIN, "Created new Async Main Node: " << *this << ".");
}

AsyncEdgeNode::AsyncEdgeNode(
        const char* name,
        const std::shared_ptr<InferenceSolutionListener>& listener)
    : AsyncEdgeNode(name, listener, dds::Participant::default_domain_id())
{
}

AsyncEdgeNode::AsyncEdgeNode(
        const std::string& name,
        const std::shared_ptr<InferenceSolutionListener>& listener)
    : AsyncEdgeNode(name.c_str(), listener, dds::Participant::default_domain_id())
{
}

AsyncEdgeNode::~AsyncEdgeNode()
{
    inference_client_->stop();
    logDebug(AMLIPCPP_NODE_ASYNCEDGE, "Destroying Async Edge Node: " << *this << ".");
}

types::TaskId AsyncEdgeNode::request_inference(
        const types::InferenceDataType& data)
{
    logDevError(AMLIPCPP_NODE_ASYNCEDGE, "Copying request data.");
    return request_inference(
        std::make_shared<types::InferenceDataType>(data)
        );
}

types::TaskId AsyncEdgeNode::request_inference(
        const std::shared_ptr<types::InferenceDataType>& data)
{
    return inference_client_->send_request_async(data);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
