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
 * @file InferenceNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/node/InferenceNode.hpp>

#include <dds/multiservice/MultiServiceServer.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

InferenceNode::InferenceNode(
        const char* name)
    : ParentNode(name, types::NodeKind::inference)
    , inference_server_(participant_->create_multiservice_server<types::InferenceDataType,
            types::InferenceSolutionDataType>(
                dds::utils::INFERENCE_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_INFERENCE, "Created new Inference Node: " << *this << ".");
}

InferenceNode::InferenceNode(
        const std::string& name)
    : InferenceNode(name.c_str())
{
}

InferenceNode::~InferenceNode()
{
    logDebug(AMLIPCPP_NODE_INFERENCE, "Destroying Inference Node: " << *this << ".");
}

void InferenceNode::process_inference(
        const std::function<types::InferenceSolutionDataType(const types::InferenceDataType&)>& callback)
{
    types::AmlipIdDataType _;
    process_inference(callback, _);
}

void InferenceNode::process_inference(
        const std::function<types::InferenceSolutionDataType(const types::InferenceDataType&)>& callback,
        types::AmlipIdDataType& client_id)
{
    change_status_(types::StateKind::running);
    auto reference = inference_server_->process_task_sync(callback);
    client_id = reference.client_id();
    change_status_(types::StateKind::stopped);
}

void InferenceNode::process_inference(
        const InferenceListener& listener)
{
    process_inference(
        [&listener]
            (const types::InferenceDataType& inference)
        {
            return listener.process_inference(inference);
        });
}

void InferenceNode::process_inference(
        const InferenceListener& listener,
        types::AmlipIdDataType& client_id)
{
    process_inference(
        [&listener]
            (const types::InferenceDataType& inference)
        {
            return listener.process_inference(inference);
        },
        client_id);
}

std::ostream& operator <<(
        std::ostream& os,
        const InferenceNode& node)
{
    os << "INFERENCE_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
