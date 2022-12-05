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
 * @file ModelManagerNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP

#include <atomic>
#include <functional>
#include <thread>

#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/node/ParentNode.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
class Reader;

template <typename T>
class Writer;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief TODO
 */
class AMLIP_CPP_DllAPI ModelListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~ModelListener() = default;

    /**
     * TODO
     */
    virtual void model_received (
            const types::ModelDataType& model) const = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that listens to Status messages.
 *
 * Status messages are sent by every Node publishing their id, kind and current state.
 * Every message of it could be processed by this Node. The use of these messages are left for the user to implement.
 * Some ideas of what to do with them:
 * - Debug purpose
 * - Store data in a DataBase that represents the state of the network and its changes.
 *
 * This Node has currently a method \c process_status_async that activates the processing of status data.
 * It can work with a Listener or by a callback that is executed with each message.
 * While this method is not called, messages will be stored in Reader.
 * Once called, each old message (except those that overflow the DDS internal history) are processed.
 *
 * @warning so far this class is not thread-safe, so \c process_status_async can only be called once, and cannot
 * be called again before calling \c stop_processing .
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ModelManagerNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI ModelManagerNode(
            const char* name,
            uint32_t domain_id);

    /**
     * @brief Construct a new Status Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI ModelManagerNode(
            const char* name);

    /**
     * @brief Destroy the Status Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~ModelManagerNode();

    AMLIP_CPP_DllAPI void start_receiving(std::shared_ptr<ModelListener> listener);

    AMLIP_CPP_DllAPI void stop_receiving();

    AMLIP_CPP_DllAPI void publish_model(types::ModelDataType& model);

protected:

    AMLIP_CPP_DllAPI void process_routine_(
            std::shared_ptr<ModelListener> listener);

    std::thread receiving_thread_;

    std::shared_ptr<dds::Reader<types::ModelDataType>> model_reader_;

    std::shared_ptr<dds::Writer<types::ModelDataType>> model_writer_;

    //! Whether the Node is currently open to receive data or it is stopped.
    std::atomic<bool> receiving_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP */
