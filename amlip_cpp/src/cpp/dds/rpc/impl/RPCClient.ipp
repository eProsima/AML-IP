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
 * @file RPCClient.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCCLIENT_IPP
#define AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCCLIENT_IPP

#include <cpp_utils/Log.hpp>
#include <cpp_utils/wait/WaitHandler.hpp>

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/direct_write.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
RPCClient<Data, Solution>::RPCClient(
        const types::AmlipIdDataType& own_id,
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler)
    : request_availability_model_(
        "rpc_request_" + topic,
        dds_handler) // REQUEST_MODEL
    , reply_available_model_(
        own_id,
        "rpc_reply_" + topic,
        dds_handler) // REPLY_MODEL
    , own_id_(own_id)
    , topic_("rpc_request_" + topic + " | rpc_reply_" + topic)
    , last_task_id_used_(0)
{
}

template <typename Data, typename Solution>
RPCClient<Data, Solution>::~RPCClient()
{
}

template <typename Data, typename Solution>
types::TaskId RPCClient<Data, Solution>::send_request(
        const Data& data,
        types::AmlipIdDataType server_id,
        uint32_t timeout /* = 0 */)
{
    // REQUEST
    types::TaskId task_id = new_task_id_();
    types::RpcRequestDataType<Data> rpc_request(own_id_, task_id, server_id, data);

    // Wait for matching
    logDebug(AMLIPCPP_DDS_RPCCLIENT, "Wait match with Reader ID: " << server_id << ".");
    eprosima::utils::event::AwakeReason reason = request_availability_model_.wait_match(server_id, timeout);

    if (reason == eprosima::utils::event::AwakeReason::disabled)
    {
        logDebug(AMLIPCPP_DDS_RPCCLIENT, "ModelManager Node " << this << " finished processing data.");

        // Break thread execution
        return task_id;
    }

    // Wait a bit to let the reader do the match
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    logDebug(AMLIPCPP_DDS_RPCCLIENT, "Matched with Reader. Seding data...");
    // Send request
    request_availability_model_.write(server_id, rpc_request);

    return task_id;
}

template <typename Data, typename Solution>
Solution RPCClient<Data, Solution>::get_reply(
        types::TaskId task_id,
        uint32_t timeout /* = 0 */)
{
    types::RpcReplyDataType<Solution> rpc_reply;
    while (true)
    {
        // WAIT FOR SOLUTION
        logDebug(AMLIPCPP_DDS_RPCCLIENT, "Waiting for reply in: " << own_id_ << ".");

        // REPLY
        eprosima::utils::event::AwakeReason reason = reply_available_model_.wait_data_available(timeout);

        if (reason == eprosima::utils::event::AwakeReason::disabled)
        {
            logDebug(AMLIPCPP_DDS_RPCCLIENT, "ModelManager Node " << this << " finished processing data.");

            // Break thread execution
            break;
        }

        try
        {
            // Read reply
            logDebug(AMLIPCPP_DDS_RPCCLIENT, "Data received. Reading data...");
            rpc_reply = reply_available_model_.read();

            // NOTE: it does not check the server, it can be assumed is the one we are waiting for
            if (rpc_reply.task_id() == task_id &&
                    rpc_reply.client_id() == own_id_)
            {
                logDebug(AMLIPCPP_DDS_RPCCLIENT, "Solution found for task: " << task_id << ".");

                break;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
    // Return the data so it is not copied but moved
    return rpc_reply.data();
}

template <typename Data, typename Solution>
types::TaskId RPCClient<Data, Solution>::new_task_id_()
{
    return last_task_id_used_++;
}

template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const RPCClient<Data, Solution>& obj)
{
    os << "RPCCLIENT{" << obj.own_id_ << ";" << obj.topic_ << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCCLIENT_IPP */
