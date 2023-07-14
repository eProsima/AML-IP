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
 * @file ModelManagerSenderNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_MODELMANAGERSENDERNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_MODELMANAGERSENDERNODE_HPP

#include <functional>

#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <amlip_cpp/node/ParentNode.hpp>

#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>
#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>


// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
class Writer;

template <typename Data, typename Solution>
class RPCServer;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief Object that listens to:
 *
 *  - new ModelDataType messages received from a \c ModelManagerReceiverNode and executes a callback.
 *
 * This class is supposed to be implemented by a User and be given to a \c ModelManagerSenderNode in order to process
 * the messages received from other Nodes in the network.
 * When data is received, \c fetch_model is called and it is expected to return a Model Solution for such data.
 */
class ModelReplier
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~ModelReplier() = default;

    /**
     * @brief Method that will be called with each ModelSolutionDataType message received to calculate an answer.
     *
     * @param data new ModelDataType message received.
     */
    virtual types::ModelSolutionDataType fetch_model (
            const types::ModelDataType data) = 0;
};

/**
 * @brief This  is a specialisation of the AML-IP node that sends
 * statistical data from models and receives requests to those models.
 */
class ModelManagerSenderNode : public ParentNode
{
public:

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     * @param domain_id DomainId of the DDS DomainParticipant
     */
    AMLIP_CPP_DllAPI ModelManagerSenderNode(
            types::AmlipIdDataType id,
            uint32_t domain_id);

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     */
    AMLIP_CPP_DllAPI ModelManagerSenderNode(
            types::AmlipIdDataType id);

    /**
     * @brief Destroy the ModelManagerSenderNode Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~ModelManagerSenderNode();

    /**
     * @brief This function copies the values in member \c statistics_
     *
     * @param name New value to be copied in member id \c statistics_.name_
     * @param data New value to be copied in member id \c statistics_.data_
     */
    void update_statistics(
            const std::string& name,
            void* data,
            const uint32_t size);

    /**
     * @brief This function gets the value of member \c statistics_ as ModelStatisticsDataType
     *
     * @return Value of member \c statistics_ as ModelStatisticsDataType
     */
    types::ModelStatisticsDataType statistics();


    /**
     * @brief Process model replies
     *
     * @throw InconsistencyException if node is already running.
     */
    void start(
            std::shared_ptr<ModelReplier> model_replier);

    /**
     * @brief Stop processing data.
     *
     * If not processing data, do nothing.
     */
    void stop();

protected:

    static eprosima::fastdds::dds::DataWriterQos default_statistics_datawriter_qos();

    /**
     * @brief Routine to be processed that read messages when available and call listener functions.
     *
     * Function to run from a thread.
     * It waits in \c statistics_writer_ to have messages matched.
     *
     * @param model_replier listener to call
     */
    void process_routine_(
            std::shared_ptr<ModelReplier> model_replier);

    /**
     * @brief Thread that waits for new data to be available.
     */
    std::thread sending_thread_;

    /**
     * @brief Reference to the Writer that writes statistics.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::Writer<types::ModelStatisticsDataType>> statistics_writer_;

    /**
     * @brief Reference to the RPC Server that sends replies.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::RPCServer<types::ModelDataType, types::ModelSolutionDataType>> model_sender_;

    //! Whether the Node is currently open to receive data or it is stopped.
    std::atomic<bool> running_;

    //! Statistical data from models.
    types::ModelStatisticsDataType statistics_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MODELMANAGERSENDERNODE_HPP */
