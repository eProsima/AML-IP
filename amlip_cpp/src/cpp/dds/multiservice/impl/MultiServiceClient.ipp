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

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICE_IMPL_READER_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICE_IMPL_READER_IPP

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <dds/network_utils/multiservice.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
MultiServiceClient<Data, Solution>::MultiServiceClient(
        const types::AmlipId& own_id,
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler)
    : own_id_(own_id)
    , topic_(topic)
    , request_availability_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::REQUEST_AVAILABILITY),
        dds_handler,
        default_request_availability_writer_qos_()) // REQUEST_AVAILABILITY
    , reply_available_reader_(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::REPLY_AVAILABLE),
        dds_handler) // REPLY_AVAILABLE
    , task_target_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::TASK_TARGET),
        dds_handler,
        default_task_target_writer_qos_()) // TASK_TARGET
    , task_data_writer_(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::TASK_DATA),
        dds_handler) // TASK_DATA
    , task_solution_reader_(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::TASK_SOLUTION),
        dds_handler) // TASK_SOLUTION
    , last_task_id_used_(0)
{
}

template <typename Data, typename Solution>
MultiServiceClient<Data, Solution>::~MultiServiceClient()
{
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos MultiServiceClient<Data, Solution>::default_request_availability_writer_qos_()
{
    eprosima::fastdds::dds::DataWriterQos qos;

    qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
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
Solution MultiServiceClient<Data, Solution>::send_request_sync(const Data& data)
{
    // SEND REQUEST AVAILABILITY
    // Get new task id
    types::TaskId this_task_id = new_task_id();

    // Create Request data
    types::MsRequestDataType request_data(own_id_, this_task_id);

    // Send request
    request_availability_writer_.publish(request_data);

    // WAIT FOR REPLY AVAILABLE
    // Wait for someone to reply
    types::MsReferenceDataType reference;

    while(true)
    {
        reply_available_reader_.wait_data_available();

        // Get task reference
        reference = reply_available_reader_.read();

        if (reference.task_id() == this_task_id ||
            reference.source_id() == own_id_)
        {
            break;
        }
    }

    // SEND TASK TARGET
    // Send the reference sent by the server
    task_target_writer_.publish(reference);

    // SEND DATA
    task_data_writer_.write(types::MsDataType<Data>(reference, data));

    // WAIT FOR SOLUTION
    Solution solution;

    while(true)
    {
        task_solution_reader_.wait_data_available();

        // Get task reference
        solution = task_solution_reader_.read();

        if (solution.task_id() == this_task_id ||
            solution.source_id() == own_id_)
        {
            break;
        }
    }

    return solution;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICE_IMPL_READER_IPP */
