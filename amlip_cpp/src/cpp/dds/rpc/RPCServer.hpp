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


template <typename Data, typename Solution>
class RequestReplier
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~RequestReplier() = default;

    /**
     * @brief Method that will be called with the request message received to calculate a reply.
     *
     * @param request new request message received.
     *
     * @return Solution to the \c requets .
     */
    virtual Solution process_request (
            Data request) = 0;
};


template <typename Data, typename Solution>
class RPCServer
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    /**
     * @brief Construct a new RPCServer object.
     *
     * @param own_id Id of the Participant (associated with the Node it belongs to)
     * @param topic Name of the topic
     * @param dds_handler
     */
    RPCServer(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    /**
     * @brief Destroy the RPCServer object
     */
    ~RPCServer();

    /**
     * @brief Disable \c request_reader_ object.
     *
     */
    void stop();

    /**
     * @brief
     *
     * @param timeout maximum wait time in milliseconds (0 = no wait)
     *
     * @return RpcRequestDataType<Data>
     */
    types::RpcRequestDataType<Data> get_request(
            uint32_t timeout = 0);

    /**
     * @brief
     *
     * @param timeout maximum wait time in milliseconds (0 = no wait)
     */
    void send_reply(
            types::RpcReplyDataType<Solution> rpc_reply,
            uint32_t timeout = 0);

protected:

    static eprosima::fastdds::dds::DataWriterQos default_rpc_datawriter_qos();
    static eprosima::fastdds::dds::DataReaderQos default_rpc_datareader_qos();

    types::TaskId new_task_id_();

    TargetedReader<types::RpcRequestDataType<Data>> request_reader_;

    DirectWriter<types::RpcReplyDataType<Solution>> reply_writer_;

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
