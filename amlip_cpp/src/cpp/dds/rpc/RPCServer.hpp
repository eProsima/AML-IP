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
 * @file RPCServer.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_RPC_RPCSERVER_HPP
#define AMLIPCPP__SRC_CPP_DDS_RPC_RPCSERVER_HPP
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
 * @brief TODO
 */
template <typename Data, typename Solution>
class RequestReplier
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~RequestReplier() = default;

    /**
     * @brief TODO
     */
    virtual Solution process_request (
            Data request) = 0;
};

/**
 * TODO
 */
template <typename Data, typename Solution>
class RPCServer
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    RPCServer(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    ~RPCServer();

    /**
     * @brief
     *
     * @param data
     * @return Solution
     *
     * @warning This method is thought to use MS in only one thread. Multithreading synchronization is not implemented.
     * Thus, using multiple threads will cause desynchronization of messages received and locks.
     */
    types::RpcRequestDataType<Data> get_request(
            uint32_t timeout = 0);

    void send_reply(
            types::RpcReplyDataType<Solution> rpc_reply,
            uint32_t timeout = 0);

protected:

    types::TaskId new_task_id_();

    TargetedReader<types::RpcRequestDataType<Data>> request_available_model_;

    DirectWriter<types::RpcReplyDataType<Solution>> reply_availability_model_;

    types::AmlipIdDataType own_id_;

    std::string topic_;
};

//! \c RPCServer to stream serializator
template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const RPCServer<Data, Solution>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/rpc/impl/RPCServer.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_RPC_RPCSERVER_HPP */
