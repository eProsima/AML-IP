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
 * @file ModelManagerSenderNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>

#include <dds/rpc/RPCClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ModelManagerSenderNode::ModelManagerSenderNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::status, types::StateKind::stopped, domain_id, dds::utils::ignore_locals_domain_participant_qos(
                name))
    , model_writer_(participant_->create_writer<types::ModelDataType>(
                dds::utils::MODEL_TOPIC_NAME,
                dds::utils::model_writer_qos()))
    , rpc_server_(
        participant_->create_rpc_server<types::ModelDataType,
        types::ModelSolutionDataType>(dds::utils::MODEL_TOPIC_NAME))
    , sending_(false)
{
    // Participant ignore local enpoints
    logInfo(AMLIPCPP_NODE_MODELMANAGER, "Created new ModelManager Node: " << *this << ".");
}

ModelManagerSenderNode::ModelManagerSenderNode(
        const char* name)
    : ModelManagerSenderNode(name, dds::Participant::default_domain_id())
{
}

ModelManagerSenderNode::~ModelManagerSenderNode()
{
    logDebug(AMLIPCPP_NODE_MODELMANAGER, "Destroying ModelManager Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (sending_)
    {
        stop();
    }

    logDebug(AMLIPCPP_NODE_MODELMANAGER, "ModelManager Node Destroyed.");
}

void ModelManagerSenderNode::start(
        std::shared_ptr<ModelReplier> replier)
{
    logInfo(AMLIPCPP_NODE_MODELMANAGER, "Start processing ModelManager messages by replier in : " << *this << ".");

    if (sending_)
    {
        throw utils::InconsistencyException(
                  STR_ENTRY << "ModelManager node " << this << " is already processing data.");
    }
    else
    {
        sending_ = true;
        sending_thread_ = std::thread(
            &ModelManagerSenderNode::process_routine_,
            this,
            replier);

        change_status_(types::StateKind::running);
    }
}

void ModelManagerSenderNode::stop()
{
    if (sending_)
    {
        sending_ = false;
        sending_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerSenderNode::process_routine_(
        std::shared_ptr<ModelReplier> replier)
{
    while (sending_)
    {
        // Call callback
        types::ModelDataType model;
        replier->model_received(model);
        ModelManagerSenderNode::publish_model(model);
    }
}

void ModelManagerSenderNode::publish_model(
        types::ModelDataType& model)
{
    model_writer_->publish(model);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
