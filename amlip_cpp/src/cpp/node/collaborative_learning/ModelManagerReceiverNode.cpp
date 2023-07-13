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
                dds::utils::MODEL_STATISTICS_TOPIC_NAME,
                default_statistics_datareader_qos()))
    , model_receiver_(
        participant_->create_rpc_client<types::ModelDataType,
        types::ModelSolutionDataType>(dds::utils::MODEL_TOPIC_NAME))
    , running_(false)
    , data_(data)
{
    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "Created new ModelManagerReceiver Node: " << *this << ".");
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        types::AmlipIdDataType id,
        types::ModelDataType& data)
    : ModelManagerReceiverNode(id, data, dds::Participant::default_domain_id())
{
    // Do nothing
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name,
        types::ModelDataType& data,
        uint32_t domain_id)
    : ModelManagerReceiverNode(types::AmlipIdDataType(name), data, domain_id)
{
    // Do nothing
}

ModelManagerReceiverNode::ModelManagerReceiverNode(
        const char* name,
        types::ModelDataType& data)
    : ModelManagerReceiverNode(name, data, dds::Participant::default_domain_id())
{
    // Do nothing
}

ModelManagerReceiverNode::~ModelManagerReceiverNode()
{
    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "Destroying ModelManagerReceiver Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (running_)
    {
        stop();
    }

    logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "ModelManagerReceiver Node Destroyed.");
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
                  STR_ENTRY << "ModelManagerReceiver node " << this << " is already processing data.");
    }
}

void ModelManagerReceiverNode::stop()
{
    if (running_.exchange(false))
    {
        statistics_reader_->stop(); // This must awake thread and it must finish
        model_receiver_->stop();
        receiving_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerReceiverNode::process_routine_(
        std::shared_ptr<ModelListener> listener)
{
    while (running_)
    {
        eprosima::amlip::types::ModelStatisticsDataType statistics;
        try
        {
            // Read statistics
            logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER, "Statistics received. Reading...");
            statistics = statistics_reader_->read();

            if (!running_)
            {
                break;
            }

            logDebug(AMLIPCPP_NODE_MODELMANAGERRECEIVER,
                    "ModelManagerReceiver Node " << *this << " read statistics :" << statistics << ".");

            if (listener->statistics_received(statistics))
            {
                // Send request
                types::TaskId task_id = model_receiver_->send_request(data_, statistics.server_id());

                // Wait reply
                eprosima::amlip::types::ModelSolutionDataType model = model_receiver_->get_reply(task_id);

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
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Finishing ModelManagerSender routine.");
}

eprosima::fastdds::dds::DataReaderQos ModelManagerReceiverNode::default_statistics_datareader_qos()
{
    eprosima::fastdds::dds::DataReaderQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    return qos;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
