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

#include <amlip_cpp/node/ParentNode.hpp>

#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>
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
 * @brief TODO
 */
class ModelListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~ModelListener() = default;

    /**
     * TODO
     */
    virtual bool statistics_received (
            const types::ModelStatisticsDataType statistics) = 0;

    /**
     * TODO
     */
    virtual bool model_received (
            const types::ModelSolutionDataType model) = 0;
};

/**
 * TODO
 */
class ModelManagerReceiverNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            types::AmlipIdDataType id,
            types::ModelDataType& data,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            types::AmlipIdDataType id,
            types::ModelDataType& data);

    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            const char* name,
            types::ModelDataType& data,
            uint32_t domain_id);

    /**
     * @brief Construct a new ModelManagerReceiverNode Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI ModelManagerReceiverNode(
            const char* name,
            types::ModelDataType& data);

    /**
     * @brief Destroy the ModelManagerReceiverNode Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~ModelManagerReceiverNode();

    void start(
            std::shared_ptr<ModelListener> listener);

    void stop();

protected:

    void process_routine_(
            std::shared_ptr<ModelListener> listener);

    std::thread receiving_thread_;

    std::shared_ptr<dds::Reader<types::ModelStatisticsDataType>> statistics_reader_;

    std::shared_ptr<dds::RPCClient<types::ModelDataType, types::ModelSolutionDataType>> model_reader_;

    //! Whether the Node is currently open to receive data or it is stopped.
    std::atomic<bool> running_;

    types::ModelDataType data_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MODELMANAGERRECEIVERNODE_HPP */
