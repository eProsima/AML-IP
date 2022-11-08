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
 * @file MultiServiceClient.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICECLIENT_IPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICECLIENT_IPP

#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/multiservice.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
MultiServiceClient<Data, Solution>::MultiServiceClient(
        const types::AmlipIdDataType& own_id,
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler)
    : request_availability_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::request_availability),
        dds_handler,
        default_request_availability_writer_qos_()) // REQUEST_AVAILABILITY
    , reply_available_reader_(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::reply_available),
        dds_handler) // REPLY_AVAILABLE
    , task_target_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_target),
        dds_handler,
        default_task_target_writer_qos_()) // TASK_TARGET
    , task_data_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_data),
        dds_handler) // TASK_DATA
    , task_solution_reader_(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_solution),
        dds_handler) // TASK_SOLUTION
    , own_id_(own_id)
    , topic_(topic)
    , last_task_id_used_(0)
{
}

template <typename Data, typename Solution>
MultiServiceClient<Data, Solution>::~MultiServiceClient()
{
}

template <typename Data, typename Solution>
Solution MultiServiceClient<Data, Solution>::send_request_sync(
        const Data& data)
{
    // SEND REQUEST AVAILABILITY
    // Get new task id
    types::TaskId this_task_id = new_task_id_();
    logDebug(AMLIPCPP_DDS_MSCLIENT, "Sending sync request " << this_task_id << " sync from " << own_id_ << ".");

    // Create Request data
    types::MsRequestDataType request_data(own_id_, this_task_id);

    // Send request
    request_availability_writer_.publish(request_data);

    // WAIT FOR REPLY AVAILABLE
    // Wait for someone to reply
    types::MsReferenceDataType reference;
    logDebug(AMLIPCPP_DDS_MSCLIENT, "Waiting for available servers in task " << request_data << ".");

    while (true)
    {
        reply_available_reader_.wait_data_available();

        // Get task reference
        reference = reply_available_reader_.read();

        if (reference.task_id() == this_task_id &&
                reference.client_id() == own_id_)
        {
            break;
        }
    }

    // From here, reference has the target of the server that is going to process the data
    logDebug(AMLIP_MULTISERVICE_CLIENT,
            "Client " << own_id_ << " sending task: " << reference.task_id() <<
            " to server: " << reference.server_id() << ".");

    // SEND TASK TARGET
    // Send the reference sent by the server
    task_target_writer_.publish(reference);

    // SEND DATA
    types::MsDataType<Data> data_(reference, data);
    task_data_writer_.write(reference.server_id(), data_);

    // WAIT FOR SOLUTION
    logDebug(AMLIPCPP_DDS_MSCLIENT, "Wait for solution of task: " << reference << ".");

    while (true)
    {
        task_solution_reader_.wait_data_available();

        // Get task reference
        types::MsDataType<Solution> ms_solution = task_solution_reader_.read();

        // NOTE: it does not check the server, it can be assumed is the one we are waiting for
        if (ms_solution.task_id() == this_task_id &&
                ms_solution.client_id() == own_id_)
        {
            logDebug(AMLIPCPP_DDS_MSCLIENT, "Solution found for task: " << reference << ".");

            // Return the data so it is not copied but moved
            return ms_solution.data();
        }
    }
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos MultiServiceClient<Data, Solution>::default_request_availability_writer_qos_()
{
    eprosima::fastdds::dds::DataWriterQos qos = utils::default_datawriter_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    // TODO this should change to Keep last 1 when keys are implemented

    return qos;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos MultiServiceClient<Data, Solution>::default_task_target_writer_qos_()
{
    return default_request_availability_writer_qos_();
}

template <typename Data, typename Solution>
types::TaskId MultiServiceClient<Data, Solution>::new_task_id_()
{
    return last_task_id_used_++;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICECLIENT_IPP */
