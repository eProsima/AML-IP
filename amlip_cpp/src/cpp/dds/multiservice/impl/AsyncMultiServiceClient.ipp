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
 * @file AsyncMultiServiceClient.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICECLIENT_IPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICECLIENT_IPP

#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/multiservice.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
AsyncMultiServiceClient<Data, Solution>::AsyncMultiServiceClient(
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
    , data_to_send_(std::make_unique<eprosima::utils::event::DBQueueWaitHandler<std::pair<std::shared_ptr<Data>, types::TaskId>>>(0, false))
    , running_(false)
    , waiter_for_request_(0, 0)
    , reply_reading_thread_(&AsyncMultiServiceClient::read_replies_, this)
    , server_to_send_(std::make_unique<eprosima::utils::event::DBQueueWaitHandler<types::MsReferenceDataType>>(0, true))
    , solution_reading_thread_(&AsyncMultiServiceClient::read_solution_, this)
    , should_stop_(false)
{
    logDebug(
        AMLIPCPP_DDS_MSASYNCCLIENT,
        "New Async MultiService Client " << *this << " created.");
}

template <typename Data, typename Solution>
AsyncMultiServiceClient<Data, Solution>::~AsyncMultiServiceClient()
{
    stop();

    // Stop thread that reads replies
    // NOTE: it is important to do it afterwards, otherwise some threads could be stuck forever
    // NOTE: this could be done as well within the stop and run, but not this time
    should_stop_ = true;
    waiter_for_request_.blocking_disable();
    reply_reading_thread_.join();
    solution_reading_thread_.join();

    logDebug(
        AMLIPCPP_DDS_MSASYNCCLIENT,
        "Async MultiService Client " << *this << " destroyed.");
}


template <typename Data, typename Solution>
void AsyncMultiServiceClient<Data, Solution>::run(std::shared_ptr<SolutionListener<Solution>> solution_listener)
{
    if (!running_.exchange(true))
    {
        solution_listener_ = solution_listener;
        data_to_send_->enable();
        send_task_request_thread_ = std::thread(&AsyncMultiServiceClient::send_request_async_routine_, this);
    }
    else
    {
        throw eprosima::utils::InconsistencyException(
                  STR_ENTRY << "AsyncMultiServiceServer node " << own_id_ << " is already processing data.");
    }
}

template <typename Data, typename Solution>
void AsyncMultiServiceClient<Data, Solution>::stop()
{
    // If it is processing data, set processing as false and wait for process to finish
    if (running_.exchange(false))
    {
        logDebug(
            AMLIPCPP_DDS_MSASYNCCLIENT,
            "Stopping sending requests in " << *this << " .");

        // Stop tasking threads when they are going to take next task
        data_to_send_->blocking_disable();

        // Wait for all those threads to stop
        send_task_request_thread_.join();
    }
}

template <typename Data, typename Solution>
types::TaskId AsyncMultiServiceClient<Data, Solution>::send_request_async(
        std::shared_ptr<Data> data)
{
    auto new_task_id = new_task_id_();

    data_to_send_->produce(std::make_pair(std::move(data), new_task_id));

    ++waiter_for_request_;

    return new_task_id;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos AsyncMultiServiceClient<Data, Solution>::default_request_availability_writer_qos_()
{
    eprosima::fastdds::dds::DataWriterQos qos = utils::default_datawriter_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    // TODO this should change to Keep last 1 when keys are implemented

    return qos;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataWriterQos AsyncMultiServiceClient<Data, Solution>::default_task_target_writer_qos_()
{
    return default_request_availability_writer_qos_();
}

template <typename Data, typename Solution>
types::TaskId AsyncMultiServiceClient<Data, Solution>::new_task_id_()
{
    return last_task_id_used_++;
}

template <typename Data, typename Solution>
void AsyncMultiServiceClient<Data, Solution>::send_request_async_routine_()
{
    // Iterate while consume is correct
    while (true)
    {
        std::shared_ptr<Data> data;
        types::TaskId this_task_id;
        try
        {
            auto task = data_to_send_->consume();
            data = std::move(task.first);
            this_task_id = task.second;
        }
        catch(const eprosima::utils::DisabledException&)
        {
            // Consumer disable, end thread
            break;
        }

        // SEND REQUEST AVAILABILITY
        // Get new task id
        logDebug(AMLIPCPP_DDS_MSCLIENT, "Sending async request " << this_task_id << " sync from " << own_id_ << ".");

        // Create Request data
        types::MsRequestDataType request_data(own_id_, this_task_id);

        // Send request
        request_availability_writer_.publish(request_data);

        // WAIT FOR REPLY AVAILABLE
        // Wait for reply thread to notify a finding
        types::MsReferenceDataType reference;
        // Wait this thread till a server is chosen
        reference = server_to_send_->consume();
        // NOTE: This algorithm disconnects a task with its thread, but as long as there is only one thread
        // nothing happens.

        // From here, reference has the target of the server that is going to process the data
        auto server = reference.server_id();
        logDebug(AMLIPCPP_MULTISERVICE_CLIENT,
                "Client " << own_id_ << " sending task: " << reference.task_id() <<
                " to server: " << server << ".");

        // SEND TASK TARGET
        // Send the reference sent by the server
        task_target_writer_.publish(reference);

        // SEND DATA
        types::MsDataType<Data> data_(reference, *data);
        task_data_writer_.write(server, data_);

        // WAIT FOR SOLUTION
        logDebug(AMLIPCPP_DDS_MSCLIENT, "Wait for solution of task: " << reference << " in other thread.");
    }
}

template <typename Data, typename Solution>
void AsyncMultiServiceClient<Data, Solution>::read_replies_()
{
    while(!should_stop_)
    {
        // Only start waiting for data once a request have been sent
        // This way it avoids to wait forever when no required
        auto reason = waiter_for_request_.wait_and_decrement();
        if (reason != eprosima::utils::event::AwakeReason::condition_met)
        {
            // Waiter has been disabled
            break;
        }

        // Read messages until the one expected (targeted to this id) is listen
        while (true)
        {
            reply_available_reader_.wait_data_available();

            // Get task reference
            types::MsReferenceDataType reference = reply_available_reader_.read();

            if (reference.client_id() != own_id_)
            {
                continue;
            }
            else if (already_chosen_tasks_.find(reference.task_id()) != already_chosen_tasks_.end())
            {
                continue;
            }
            else
            {
                logDebug(AMLIPCPP_DDS_MSCLIENT, "Received task reference for this client: " << reference << ".");

                already_chosen_tasks_.insert(reference.task_id());

                // Notify task thread that a new server is found for task
                server_to_send_->produce(reference);
                break;
            }
        }
    }
}

template <typename Data, typename Solution>
void AsyncMultiServiceClient<Data, Solution>::read_solution_()
{
    while(!should_stop_)
    {
        // TODO: this should not be done
        uint32_t timeout = 1000;
        auto reason = task_solution_reader_.wait_data_available(timeout);
        if (reason != eprosima::utils::event::AwakeReason::condition_met)
        {
            continue;
        }

        // Get task reference
        types::MsDataType<Solution> ms_solution = task_solution_reader_.read();

        logDebug(AMLIPCPP_DDS_MSCLIENT, "Solution found for task: " << ms_solution.task_id() << ".");

        // Return the data so it is not copied but moved
        solution_listener_->solution_received(
            std::make_unique<Solution>(ms_solution.data()),
            ms_solution.task_id(),
            ms_solution.client_id(),
            ms_solution.server_id());
    }
}

template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const AsyncMultiServiceClient<Data, Solution>& obj)
{
    os << "ASYNC_MSCLIENT{" << obj.own_id_ << ";" << obj.topic_ << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICECLIENT_IPP */
