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

#include <cpp_utils/utils.hpp>
#include <cpp_utils/types/cast.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>

#include <dds/rpc/RPCClient.hpp>
#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>
#include <dds/network_utils/model_manager.hpp>

namespace eprosima {
namespace amlip {
namespace node {


ModelManagerSenderNode::ModelManagerSenderNode(
        types::AmlipIdDataType id,
        uint32_t domain_id)
    : ParentNode(id, types::NodeKind::model_sender, types::StateKind::stopped, domain_id, dds::utils::ignore_locals_domain_participant_qos(
                id.name().c_str()))
    , statistics_writer_(participant_->create_writer<types::ModelStatisticsDataType>(
                dds::utils::MODEL_STATISTICS_TOPIC_NAME,
                default_statistics_datawriter_qos()))
    , model_sender_(
        participant_->create_rpc_server<types::ModelRequestDataType,
        types::ModelReplyDataType>(dds::utils::MODEL_TOPIC_NAME))
    , running_(false)
{
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Created new ModelManager Node: " << *this << ".");
}

ModelManagerSenderNode::ModelManagerSenderNode(
        types::AmlipIdDataType id)
    : ModelManagerSenderNode(id, dds::Participant::default_domain_id())
{
    // Do nothing
}

ModelManagerSenderNode::~ModelManagerSenderNode()
{
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Destroying ModelManagerSender Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (running_)
    {
        stop();
    }

    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "ModelManagerSender Node Destroyed.");
}

void ModelManagerSenderNode::publish_statistics(
        const std::string& name,
        void* data,
        const uint32_t size)
{
    //! Statistical data from models.
    types::ModelStatisticsDataType statistics(name, data, size);

    statistics.server_id(participant_->id());

    // Send statistics
    statistics_writer_->publish(statistics);
}

void ModelManagerSenderNode::publish_statistics(
        const std::string& name,
        const std::vector<types::ByteType>& data)
{
    publish_statistics(
        name,
        utils::copy_to_void_ptr(utils::cast_to_void_ptr(data.data()), data.size()),
        data.size());
}

void ModelManagerSenderNode::publish_statistics(
        const std::string& name,
        const std::string& data)
{
    publish_statistics(
        name,
        utils::copy_to_void_ptr(utils::cast_to_void_ptr(data.c_str()), data.length()),
        data.length());
}

void ModelManagerSenderNode::start(
        std::shared_ptr<ModelReplier> model_replier)
{
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER,
            "Start processing ModelManager messages by replier in : " << *this << ".");

    if (!running_.exchange(true))
    {
        change_status_(types::StateKind::running);

        sending_thread_ = std::thread(
            &ModelManagerSenderNode::process_routine_,
            this,
            model_replier);
    }
    else
    {
        throw utils::InconsistencyException(
                  STR_ENTRY << "ModelManagerSender node " << this << " is already processing data.");
    }
}

void ModelManagerSenderNode::stop()
{
    if (running_.exchange(false))
    {
        model_sender_->stop();
        sending_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerSenderNode::process_routine_(
        std::shared_ptr<ModelReplier> model_replier)
{
    while (running_)
    {
        // Wait request
        types::RpcRequestDataType<types::ModelRequestDataType> request =
                model_sender_->get_request();

        if (!running_)
        {
            break;
        }

        // Call callback
        eprosima::amlip::types::ModelReplyDataType solution =
                model_replier->fetch_model(request.data());

        types::RpcReplyDataType<types::ModelReplyDataType> reply(
            request.client_id(),
            request.task_id(),
            participant_->id(),
            std::move(solution));

        // Send reply
        model_sender_->send_reply(reply);

    }
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Finishing ModelManagerSender routine.");
}

eprosima::fastdds::dds::DataWriterQos ModelManagerSenderNode::default_statistics_datawriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    return qos;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
