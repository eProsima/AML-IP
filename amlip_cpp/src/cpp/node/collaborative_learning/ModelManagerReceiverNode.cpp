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
 * @file ModelManagerReceiverNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>

#include <dds/rpc/RPCClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::model_receiver, types::StateKind::stopped, domain_id, dds::utils::ignore_locals_domain_participant_qos(
                name))
    , model_reader_(participant_->create_reader<types::ModelDataType>(
                dds::utils::MODEL_TOPIC_NAME,
                dds::utils::model_reader_qos()))
    , rpc_client_(
        participant_->create_rpc_client<types::ModelDataType,
        types::ModelSolutionDataType>(dds::utils::MODEL_TOPIC_NAME))
    , receiving_(false)
{
    // Participant ignore local enpoints
    logInfo(AMLIPCPP_NODE_MODELMANAGER, "Created new ModelManagerReceiver Node: " << *this << ".");
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name)
    : ModelManagerReceiverNode(name, dds::Participant::default_domain_id())
{
}

ModelManagerReceiverNode::~ModelManagerReceiverNode()
{
    logDebug(AMLIPCPP_NODE_MODELMANAGER, "Destroying ModelManagerReceiver Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (receiving_)
    {
        stop();
    }

    logDebug(AMLIPCPP_NODE_MODELMANAGER, "ModelManager Node Destroyed.");
}

void ModelManagerReceiverNode::start(
        std::shared_ptr<ModelListener> listener)
{
    logInfo(AMLIPCPP_NODE_MODELMANAGER, "Start processing ModelManager messages by listener in : " << *this << ".");

    if (receiving_)
    {
        throw utils::InconsistencyException(
                  STR_ENTRY << "ModelManager node " << this << " is already processing data.");
    }
    else
    {
        receiving_ = true;
        receiving_thread_ = std::thread(
            &ModelManagerReceiverNode::process_routine_,
            this,
            listener);

        change_status_(types::StateKind::running);
    }
}

void ModelManagerReceiverNode::stop()
{
    if (receiving_)
    {
        receiving_ = false;
        model_reader_->awake_waiting_threads(); // This must awake thread and it must finish
        receiving_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerReceiverNode::process_routine_(
        std::shared_ptr<ModelListener> listener)
{
    while (receiving_)
    {
        // Wait for data
        utils::event::AwakeReason reason = model_reader_->wait_data_available();

        if (reason == utils::event::AwakeReason::disabled)
        {
            logDebug(AMLIPCPP_NODE_MODELMANAGER, "ModelManager Node " << *this << " finished processing data.");

            // Break thread execution
            return;
        }

        // Read data
        types::ModelDataType model = model_reader_->read();

        logDebug(AMLIPCPP_NODE_MODELMANAGER, "ModelManager Node " << *this << " read data :" << model << ".");

        // Call callback
        listener->model_received(model);
    }
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
