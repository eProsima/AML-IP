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
 * @file MultiServiceServer.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICESERVER_IPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICESERVER_IPP

#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/multiservice.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
MultiServiceServer<Data, Solution>::MultiServiceServer(
        const types::AmlipIdDataType& own_id,
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler)
    : request_availability_reader_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::request_availability),
        dds_handler,
        default_request_availability_reader_qos_()) // REQUEST_AVAILABILITY
    , reply_available_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::reply_available),
        dds_handler) // REPLY_AVAILABLE
    , task_target_reader_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_target),
        dds_handler,
        default_task_target_reader_qos_()) // TASK_TARGET
    , task_data_reader_(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_data),
        dds_handler) // TASK_DATA
    , task_solution_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_solution),
        dds_handler) // TASK_SOLUTION
    , own_id_(own_id)
    , topic_(topic)
{
}

template <typename Data, typename Solution>
MultiServiceServer<Data, Solution>::~MultiServiceServer()
{
}

template <typename Data, typename Solution>
types::MsReferenceDataType MultiServiceServer<Data, Solution>::process_task_sync(
        std::function<Solution(const Data&)> process_callback)
{
    types::MsReferenceDataType task_target;
    logDebug(AMLIPCPP_DDS_MSSERVER, "Waiting for request in: " << own_id_ << ".");

    while (true)
    {
        // WAIT FOR REQUEST AVAILABILITY
        request_availability_reader_.wait_data_available();

        // read request
        types::MsRequestDataType request = request_availability_reader_.read();

        // If this task is already known to have been targeted, skip it
        if (already_processed_requests_.find(request) != already_processed_requests_.end())
        {
            continue;
        }

        logDebug(AMLIPCPP_DDS_MSSERVER, "Request received: " << request << ". Sending available reply.");

        // If this task is already known to have been targeted, skip it
        if (already_processed_requests_.find(request) != already_processed_requests_.end())
        {
            continue;
        }

        // ANSWER AVAILABLE
        // Create data
        types::MsReferenceDataType reference(
            request.client_id(),
            request.task_id(),
            own_id_);

        // Write data
        reply_available_writer_.write(reference.client_id(), reference);

        // WAIT FOR TASK TARGET
        while (true)
        {
            // Wait for data to arrive
            task_target_reader_.wait_data_available();

            // Read data
            task_target = task_target_reader_.read();

            // Add it as already processed requests so it is not possible to wait for it again
            already_processed_requests_.insert(task_target.request());

            // Check if it is the reference we are expecting
            if (task_target.client_id() != reference.client_id() ||
                    task_target.task_id() != reference.task_id())
            {
                // If not, continue waiting
                logDebug(AMLIPCPP_DDS_MSSERVER, "Request received but not valid: " << task_target << ". Keep waiting.");
                continue;
            }
            else
            {
                // Break loop. Out loop it will be check if this is the target
                break;
            }
        }

        if (task_target.client_id() == reference.client_id() && task_target.server_id() == own_id_)
        {
            // This is the target, stop waiting
            break;
        }
        else
        {
            logDebug(
                AMLIPCPP_DDS_MSSERVER,
                "Task: " << request << " lost due to other server has been taken. Start over.");
        }
    }

    // From here, task_target has the reference for the task this must solve
    logDebug(AMLIPCPP_MULTISERVICE_SERVER,
            "Server " << own_id_ << " processing task: " << task_target.task_id() <<
            " from client: " << task_target.client_id() << ".");

    // WAIT FOR TASK DATA
    task_data_reader_.wait_data_available();
    types::MsDataType<Data> ms_data = task_data_reader_.read();


    // PROCESS DATA AND SEND SOLUTION

    // Process solution from callback
    Solution solution = process_callback(ms_data.data());

    // Create solution data
    types::MsDataType<Solution> ms_solution(
        types::MsReferenceDataType(task_target),
        std::move(solution));

    // Send solution
    task_solution_writer_.write(task_target.client_id(), ms_solution);

    return task_target;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataReaderQos MultiServiceServer<Data, Solution>::default_request_availability_reader_qos_()
{
    eprosima::fastdds::dds::DataReaderQos qos = utils::default_datareader_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    // TODO this should change to Keep last 1 when keys are implemented

    return qos;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataReaderQos MultiServiceServer<Data, Solution>::default_task_target_reader_qos_()
{
    return default_request_availability_reader_qos_();
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_MULTISERVICESERVER_IPP */
