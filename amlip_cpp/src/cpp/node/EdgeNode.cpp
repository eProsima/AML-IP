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
 * @file EdgeNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <dds/multiservice/MultiServiceClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/EdgeNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

EdgeNode::EdgeNode(
        const char* name)
    : ParentNode(name, types::NodeKind::edge)
    , inference_client_(participant_->create_multiservice_client<types::InferenceDataType,
            types::InferenceSolutionDataType>(
                dds::utils::INFERENCE_TOPIC_NAME))
{
    logInfo(AMLIPCPP_NODE_EDGE, "Created new Edge Node: " << *this << ".");
}

EdgeNode::EdgeNode(
        const std::string& name)
    : EdgeNode(name.c_str())
{
}

EdgeNode::~EdgeNode()
{
    logDebug(AMLIPCPP_NODE_EDGE, "Destroying Edge Node: " << *this << ".");
}

types::InferenceSolutionDataType EdgeNode::request_inferred(
        const types::InferenceDataType& data)
{
    types::AmlipIdDataType _;
    return request_inferred(data, _);
}

types::InferenceSolutionDataType EdgeNode::request_inferred(
        const types::InferenceDataType& data,
        types::AmlipIdDataType& id)
{
    change_status_(types::StateKind::running);
    types::InferenceSolutionDataType solution = inference_client_->send_request_sync(data, id);
    change_status_(types::StateKind::stopped);
    return solution;
}

std::ostream& operator <<(
        std::ostream& os,
        const EdgeNode& node)
{
    os << "EDGE_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
