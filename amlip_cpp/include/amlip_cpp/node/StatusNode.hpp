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
 * @file StatusNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP

#include <atomic>
#include <functional>
#include <thread>

#include <amlip_cpp/types/status/StatusDataType.hpp>
#include <amlip_cpp/node/ParentNode.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
class Reader;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief Object that listens to new Status messages received from a \c StatusNode and executes a callback.
 *
 * This class is supposed to be implemented by a User and be given to a \c StatusNode in order to process
 * Status messages received from other Nodes in the network.
 * Every Status message call \c status_received with the message as argument.
 */
class AMLIP_CPP_DllAPI StatusListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~StatusListener() = default;

    /**
     * @brief Method that will be called with each Status message received
     *
     * @param status new Status message received.
     */
    virtual void status_received (
            const types::StatusDataType& status) const = 0;
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
class StatusNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI StatusNode(
            const char* name,
            uint32_t domain_id);

    /**
     * @brief Construct a new Status Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI StatusNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI StatusNode(
            const std::string& name);

    /**
     * @brief Destroy the Status Node object
     *
     * If it is processing data, it stops.
     */
    AMLIP_CPP_DllAPI ~StatusNode();

    /**
     * Execute in a new thread a passive listening in Status topic and execute the callback given with each message.
     *
     * @pre Should only be called once per instance before calling \c stop_processing
     *
     * @warning this does not own the callback, so it must be kept alive as long as it could be called.
     */
    AMLIP_CPP_DllAPI void process_status_async(
            const std::function<void(const types::StatusDataType&)>& callback);

    /**
     * Execute in a new thread a passive listening in Status topic and execute method \c status_received
     * of the given Listener with each message.
     *
     * It calls \c process_routine_ with a new created function that calls the internal listener.
     *
     * @pre Should only be called once per instance before calling \c stop_processing
     *
     * @warning this does not own the Listener, so the Listener must be kept alive as long as it could be called.
     */
    AMLIP_CPP_DllAPI void process_status_async(
            const StatusListener& callback_functor);

    /**
     * Stop the internal thread that is running created in \c process_status_async
     */
    AMLIP_CPP_DllAPI void stop_processing();

protected:

    /**
     * @brief Routine to be processed asynchronously that read message when available and call callback.
     *
     * Function to run from a thread.
     * It waits in \c status_reader_ to have messages to read, and for each message it reads it and pass it
     * to the callback.
     *
     * @param callback function that will be called with each Status message received.
     */
    AMLIP_CPP_DllAPI void process_routine_(
            const std::function<void(const types::StatusDataType&)>& callback);

    /**
     * @brief Thread that waits for new data to be available. Each received message is read and passed to callback.
     *
     * @todo Move this to a thread pool
     */
    std::thread process_thread_;

    /**
     * @brief Reference to the DataReader that reads Status messages.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::Reader<types::StatusDataType>> status_reader_;

    //! Whether the Node is currently processing data or it is stopped.
    std::atomic<bool> processing_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP */
