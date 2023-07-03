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
 * @file RPCServer.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCSERVER_IPP
#define AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCSERVER_IPP

#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
RPCServer<Data, Solution>::RPCServer(
        const types::AmlipIdDataType& own_id,
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler)
    : request_available_model_(
        own_id,
        topic,
        dds_handler) // REQUEST_AVAILABILITY
    , reply_availability_model_(
        topic,
        dds_handler) // REPLY_AVAILABLE
    , own_id_(own_id)
    , topic_(topic)
{
}

template <typename Data, typename Solution>
RPCServer<Data, Solution>::~RPCServer()
{
}

template <typename Data, typename Solution>
types::TaskId RPCServer<Data, Solution>::send_reply(
        std::function<Solution(const Data&)> process_callback)
{
    // WAIT FOR TASK DATA
    request_available_model_.wait_data_available();
    types::RpcRequestDataType<Data> rpc_request = request_available_model_.read();


    // PROCESS DATA AND SEND SOLUTION

    // Process solution from callback
    Solution solution = process_callback(rpc_request.data());

    // Create solution data
    types::RpcReplyDataType<Solution> rpc_solution(
        rpc_request.client_id(),
        rpc_request.task_id(),
        own_id_,
        std::move(solution));

    // Send solution
    reply_availability_model_.write(rpc_request.client_id(), rpc_solution);

    return rpc_request.task_id();
}

template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const RPCServer<Data, Solution>& obj)
{
    os << "RPCCLIENT{" << obj.own_id_ << ";" << obj.topic_ << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCSERVER_IPP */
