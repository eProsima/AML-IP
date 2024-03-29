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
 * @file ModelManagerReceiverNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_MODELMANAGERRECEIVERNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_MODELMANAGERRECEIVERNODE_HPP

#include <functional>

#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <amlip_cpp/node/ParentNode.hpp>

#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/model/ModelReplyDataType.hpp>
#include <amlip_cpp/types/model/ModelRequestDataType.hpp>
#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>


// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
class RPCClient;

template <typename T>
class Reader;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief Object that listens to:
 *
 *  - new ModelStatisticsDataType messages received from a \c ModelManagerSenderNode and executes a callback.
 *  - new ModelReplyDataType messages received from a \c ModelManagerSenderNode and executes a callback.
 *
 * This class is supposed to be implemented by a User and be given to a \c ModelManagerReceiverNode in order to process
 * the messages received from other Nodes in the network.
 */
class ModelListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~ModelListener() = default;

    /**
     * @brief Method that will be called with each ModelStatisticsDataType message received
     *
     * @param statistics new ModelStatisticsDataType message received.
     */
    virtual bool statistics_received (
            const types::ModelStatisticsDataType statistics) = 0;

    /**
     * @brief Method that will be called with each ModelReplyDataType message received
     *
     * @param model new ModelReplyDataType message received.
     */
    virtual bool model_received (
            const types::ModelReplyDataType model) = 0;
};

/**
 * @brief This  is a specialisation of the AML-IP node that receives
 * statistical data from models and sends requests to those models
 * depending on the data received.
 */
class ModelManagerReceiverNode : public ParentNode
{
public:

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     * @param data
     * @param domain_id DomainId of the DDS DomainParticipant
     */
    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            types::AmlipIdDataType id,
            types::ModelRequestDataType& data,
            uint32_t domain_id);

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param id Id of the Participant (associated with the Node it belongs to)
     * @param data
     */
    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            types::AmlipIdDataType id,
            types::ModelRequestDataType& data);

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     * @param data
     * @param domain_id DomainId of the DDS DomainParticipant
     */
    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            const char* name,
            types::ModelRequestDataType& data,
            uint32_t domain_id);

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     * @param data
     */
    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            const char* name,
            types::ModelRequestDataType& data);

    /**
     * @brief Destroy the ModelManagerReceiverNode Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~ModelManagerReceiverNode();

    /**
     * @brief Process model requests
     *
     * @throw InconsistencyException if node is already running.
     */
    void start(
            std::shared_ptr<ModelListener> listener);

    /**
     * @brief Stop processing data.
     *
     * If not processing data, do nothing.
     */
    void stop();

protected:

    static eprosima::fastdds::dds::DataReaderQos default_statistics_datareader_qos();

    /**
     * @brief Routine to be processed that read messages when available and call listener functions.
     *
     * Function to run from a thread.
     * It waits in \c statistics_reader_ to have messages to read.
     *
     * @param listener listener to call
     */
    void process_routine_(
            std::shared_ptr<ModelListener> listener);

    /**
     * @brief Thread that waits for new data to be available.
     */
    std::thread receiving_thread_;

    /**
     * @brief Reference to the Reader that reads statistics.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::Reader<types::ModelStatisticsDataType>> statistics_reader_;

    /**
     * @brief Reference to the RPC Client that sends request.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::RPCClient<types::ModelRequestDataType, types::ModelReplyDataType>> model_receiver_;

    //! Whether the Node is currently open to receive data or it is stopped.
    std::atomic<bool> running_;

    //! Data to request to ModelManagerSenderNode.
    types::ModelRequestDataType data_;

    //! Maximum wait reply in milliseconds (0 = no wait)
    static const uint32_t REPLY_TIMEOUT_;        // 2500
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MODELMANAGERRECEIVERNODE_HPP */
