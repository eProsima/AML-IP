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
 * @file ModelManagerNode.cpp
 */

#include <cpp_utils/Log.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>

#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ModelManagerNode::ModelManagerNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::status, types::StateKind::stopped, domain_id)
    , model_reader_(participant_->create_reader<types::ModelDataType>(
                dds::utils::MODEL_TOPIC_NAME,
                dds::utils::model_reader_qos()))
    , model_writer_(participant_->create_writer<types::ModelDataType>(
                dds::utils::MODEL_TOPIC_NAME,
                dds::utils::model_writer_qos()))
    , receiving_(false)
{
    logInfo(AMLIPCPP_NODE_MODELMANAGER, "Created new ModelManager Node: " << *this << ".");
}

ModelManagerNode::ModelManagerNode(
        const char* name)
    : ModelManagerNode(name, dds::Participant::default_domain_id())
{
}

ModelManagerNode::~ModelManagerNode()
{
    logDebug(AMLIPCPP_NODE_MODELMANAGER, "Destroying ModelManager Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (receiving_)
    {
        stop_receiving();
    }

    logDebug(AMLIPCPP_NODE_MODELMANAGER, "ModelManager Node Destroyed.");
}

void ModelManagerNode::start_receiving(
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
            &ModelManagerNode::process_routine_,
            this,
            listener);

        change_status_(types::StateKind::running);
    }
}

void ModelManagerNode::stop_receiving()
{
    if (receiving_)
    {
        receiving_ = false;
        model_reader_->awake_waiting_threads(); // This must awake thread and it must finish
        receiving_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerNode::process_routine_(
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

void ModelManagerNode::publish_model(types::ModelDataType& model)
{
    model_writer_->publish(model);
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
