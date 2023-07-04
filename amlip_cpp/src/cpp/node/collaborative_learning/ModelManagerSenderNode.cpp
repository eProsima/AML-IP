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
#include <dds/network_utils/model_manager.hpp>

namespace eprosima {
namespace amlip {
namespace node {


ModelManagerSenderNode::ModelManagerSenderNode(
        types::AmlipIdDataType id,
        types::ModelStatisticsDataType& statistics,
        uint32_t domain_id)
    : ParentNode(id, types::NodeKind::model_sender, types::StateKind::stopped, domain_id, dds::utils::ignore_locals_domain_participant_qos(
                id.name().c_str()))
    , statistics_writer_(participant_->create_writer<types::ModelStatisticsDataType>(
                dds::utils::MODEL_STATISTICS_TOPIC_NAME))
    , model_writer_(
        participant_->create_rpc_server<types::ModelDataType,
        types::ModelSolutionDataType>(dds::utils::MODEL_TOPIC_NAME))
    , running_(false)
    , statistics_(statistics)
{
    statistics_.server_id(participant_->id());
    // Participant ignore local enpoints
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Created new ModelManager Node: " << *this << ".");
}

ModelManagerSenderNode::ModelManagerSenderNode(
        types::AmlipIdDataType id,
        types::ModelStatisticsDataType& statistics)
    : ModelManagerSenderNode(id, statistics, dds::Participant::default_domain_id())
{
}

ModelManagerSenderNode::~ModelManagerSenderNode()
{
    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Destroying ModelManager Node: " << *this << ".");

    // If the thread is running, stop it and join thread
    if (running_)
    {
        stop();
    }

    logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "ModelManager Node Destroyed.");
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
                  STR_ENTRY << "ModelManager node " << this << " is already processing data.");
    }
}

void ModelManagerSenderNode::stop()
{
    if (running_.exchange(false))
    {
        sending_thread_.join();

        change_status_(types::StateKind::stopped);
    }
}

void ModelManagerSenderNode::process_routine_(
        std::shared_ptr<ModelReplier> model_replier)
{
    while (running_)
    {
        // Wait for discover a reader
        utils::event::AwakeReason reason = statistics_writer_->wait_match(dds::utils::WAIT_MS);

        if (reason == utils::event::AwakeReason::disabled)
        {
            logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "ModelManager Node " << *this << " finished processing data.");

            // Break thread execution
            return;
        }

        // Wait a bit to let the reader do the match
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        if (statistics_writer_->readers_matched() > 0)
        {
            logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER, "Writer has matched. Sending statistics...");

            // Send statistics
            statistics_writer_->publish(statistics_);

            logDebug(AMLIPCPP_DDS_MODELMANAGERSENDER,
                    "ModelManager Node " << *this << " send statistics :" << statistics_ << ".");

            types::RpcRequestDataType<types::ModelDataType> request =
                    model_writer_->get_request(dds::utils::WAIT_MS);

            logDebug(AMLIPCPP_MANUAL_TEST, "Wait match with Reader ID: " << request.client_id() << ".");
            eprosima::amlip::types::ModelSolutionDataType solution = model_replier->send_model(request.data());

            types::RpcReplyDataType<types::ModelSolutionDataType> reply(
                request.client_id(),
                request.task_id(),
                participant_->id(),
                std::move(solution));

            // Call callback to send model
            model_writer_->send_reply(reply, dds::utils::WAIT_MS);
        }

    }
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
