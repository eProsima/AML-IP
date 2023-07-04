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
#include <dds/network_utils/model_manager.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

ModelManagerReceiverNode::ModelManagerReceiverNode(
        types::AmlipIdDataType id,
        types::ModelDataType& data,
        uint32_t domain_id)
    : ParentNode(id, types::NodeKind::model_receiver, types::StateKind::stopped, domain_id, dds::utils::ignore_locals_domain_participant_qos(
                id.name().c_str()))
    , statistics_reader_(participant_->create_reader<types::ModelStatisticsDataType>(
                dds::utils::MODEL_STATISTICS_TOPIC_NAME))
    , model_reader_(
        participant_->create_rpc_client<types::ModelDataType,
        types::ModelSolutionDataType>(dds::utils::MODEL_TOPIC_NAME))
    , running_(false)
    , data_(data)
{
    // Participant ignore local enpoints
    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVERRECEIVER, "Created new ModelManagerReceiver Node: " << *this << ".");
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        types::AmlipIdDataType id,
        types::ModelDataType& data)
    : ModelManagerReceiverNode(id, data, dds::Participant::default_domain_id())
{
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name,
        types::ModelDataType& data,
        uint32_t domain_id)
    : ModelManagerReceiverNode(types::AmlipIdDataType(name), data, domain_id)
{
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name,
        types::ModelDataType& data)
    : ModelManagerReceiverNode(name, data, dds::Participant::default_domain_id())
{
}

ModelManagerReceiverNode::~ModelManagerReceiverNode()
{
    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "Destroying ModelManagerReceiver Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (running_)
    {
        stop();
    }

    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "ModelManager Node Destroyed.");
}

void ModelManagerReceiverNode::start(
        std::shared_ptr<ModelListener> listener)
{
    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER,
            "Start processing ModelManager messages by listener in : " << *this << ".");

    if (!running_.exchange(true))
    {
        change_status_(types::StateKind::running);

        receiving_thread_ = std::thread(
            &ModelManagerReceiverNode::process_routine_,
            this,
            listener);
    }
    else
    {
        throw utils::InconsistencyException(
                  STR_ENTRY << "ModelManager node " << this << " is already processing data.");
    }
}

void ModelManagerReceiverNode::stop()
{
    if (running_.exchange(false))
    {
        statistics_reader_->awake_waiting_threads(); // This must awake thread and it must finish
        receiving_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerReceiverNode::process_routine_(
        std::shared_ptr<ModelListener> listener)
{
    while (running_)
    {
        // Wait for data
        utils::event::AwakeReason reason = statistics_reader_->wait_data_available(dds::utils::WAIT_MS);

        if (reason == utils::event::AwakeReason::disabled)
        {
            logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "ModelManager Node " << *this << " finished processing data.");
            return;

            // Break thread execution
            return;
        }

        // Read data
        eprosima::amlip::types::ModelStatisticsDataType statistics;
        try
        {
            // Read statistics
            logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "Statistics received. Reading...");

            statistics = statistics_reader_->read();

            logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER,
                    "ModelManager Node " << *this << " read statistics :" << statistics << ".");

            if (listener->statistics_received(statistics))
            {
                types::TaskId task_id = model_reader_->send_request(data_, statistics.server_id(), dds::utils::WAIT_MS);

                eprosima::amlip::types::ModelSolutionDataType model = model_reader_->get_reply(task_id,
                                dds::utils::WAIT_MS);

                // Call callback
                listener->model_received(model);

                return;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
