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

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>

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
 * @brief TODO
 */
class AMLIP_CPP_DllAPI ModelReplier
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~ModelReplier() = default;

    /**
     * TODO
     */
    virtual void model_received (
            const types::ModelSolutionDataType& model) const = 0;
};

/**
 * TODO
 */
class ModelManagerSenderNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI ModelManagerSenderNode(
            const char* name,
            uint32_t domain_id);

    /**
     * @brief Construct a new ModelManagerSenderNode Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI ModelManagerSenderNode(
            const char* name);

    /**
     * @brief Destroy the ModelManagerSenderNode Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~ModelManagerSenderNode();

    void start(
            std::shared_ptr<ModelReplier> replier);

    void stop();

    void publish_model(
            types::ModelDataType& model);

protected:

    void process_routine_(
            std::shared_ptr<ModelReplier> replier);

    std::thread sending_thread_;

    std::shared_ptr<dds::Writer<types::ModelDataType>> model_writer_;

    std::shared_ptr<dds::RPCServer<types::ModelDataType, types::ModelSolutionDataType>> rpc_server_;

    //! Whether the Node is currently open to receive data or it is stopped.
    std::atomic<bool> sending_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MODELMANAGERSENDERNODE_HPP */
