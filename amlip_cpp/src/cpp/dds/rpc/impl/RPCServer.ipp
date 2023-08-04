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
    : request_reader_(
        own_id,
        "rpc_request_" + topic,
        dds_handler,
        default_rpc_datareader_qos()) // REQUEST
    , reply_writer_(
        "rpc_reply_" + topic,
        dds_handler,
        default_rpc_datawriter_qos()) // REPLY
    , own_id_(own_id)
    , topic_("rpc_request_" + topic + " | rpc_reply_" + topic)
{
}

template <typename Data, typename Solution>
RPCServer<Data, Solution>::~RPCServer()
{
}

template <typename Data, typename Solution>
void RPCServer<Data, Solution>::stop()
{
    request_reader_.stop();
}

template <typename Data, typename Solution>
types::RpcRequestDataType<Data> RPCServer<Data, Solution>::get_request(
        uint32_t timeout /* = 0 */)
{
    types::RpcRequestDataType<Data> rpc_request;
    while (true)
    {
        // Wait for request
        logDebug(AMLIPCPP_DDS_RPCSERVER, "Waiting for request in: " << own_id_ << ".");
        eprosima::utils::event::AwakeReason reason = request_reader_.wait_data_available(timeout);

        if (reason == eprosima::utils::event::AwakeReason::disabled)
        {
            logDebug(AMLIPCPP_DDS_RPCSERVER, *this << " finished processing data.");

            // Break thread execution
            break;
        }

        try
        {
            // Read request
            logDebug(AMLIPCPP_DDS_RPCSERVER, "Data received. Reading data...");
            rpc_request = request_reader_.read();
            break;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
    // Return the request
    return rpc_request;
}

template <typename Data, typename Solution>
void RPCServer<Data, Solution>::send_reply(
        types::RpcReplyDataType<Solution> rpc_reply,
        uint32_t timeout /* = 0 */)
{
    // Send reply
    reply_writer_.write(rpc_reply.client_id(), rpc_reply);

    logDebug(AMLIPCPP_DDS_RPCSERVER, "Direct Writer has sent message: " << rpc_reply.data() << ".");
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos RPCServer<Data, Solution>::default_rpc_datawriter_qos()
{
    eprosima::fastdds::dds::DataWriterQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    return qos;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataReaderQos RPCServer<Data, Solution>::default_rpc_datareader_qos()
{
    eprosima::fastdds::dds::DataReaderQos qos;

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = 1;

    return qos;
}

template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const RPCServer<Data, Solution>& obj)
{
    os << "RPCSERVER{" << obj.own_id_ << ";" << obj.topic_ << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_RPC_IMPL_RPCSERVER_IPP */
