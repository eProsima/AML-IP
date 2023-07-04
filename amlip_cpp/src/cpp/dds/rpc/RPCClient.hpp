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
 * @file RPCClient.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS__RPC_RPCCLIENT_HPP
#define AMLIPCPP__SRC_CPP_DDS__RPC_RPCCLIENT_HPP

#include <dds/DdsHandler.hpp>
#include <dds/DirectWriter.hpp>
#include <dds/TargetedReader.hpp>

#include <types/rpc/RpcRequestDataType.hpp>
#include <types/rpc/RpcReplyDataType.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * TODO
 */
template <typename Data, typename Solution>
class RPCClient
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    RPCClient(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    ~RPCClient();

    /**
     * @brief
     *
     * @param data [in] Data to send in request
     * @param server [out] Id of the server that has answered the data
     *
     * @return Solution
     *
     * @warning This method is thought to use MS in only one thread. Multithreading synchronization is not implemented.
     * Thus, using multiple threads will cause desynchronization of messages received and locks.
     */
    types::TaskId send_request(
            const Data& data,
            types::AmlipIdDataType server,
            uint32_t timeout = 0);

    Solution get_reply(
            types::TaskId task_id,
            uint32_t timeout = 0);

protected:

    types::TaskId new_task_id_();

    DirectWriter<types::RpcRequestDataType<Data>> request_availability_model_;

    TargetedReader<types::RpcReplyDataType<Solution>> reply_available_model_;

    types::AmlipIdDataType own_id_;

    std::string topic_;

    std::atomic<types::TaskId> last_task_id_used_;
};

//! \c RPCClient to stream serializator
template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const RPCClient<Data, Solution>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/rpc/impl/RPCClient.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS__RPC_RPCCLIENT_HPP */
