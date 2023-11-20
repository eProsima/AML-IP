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
 * @file AsyncMultiServiceServer.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICESERVER_IPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICESERVER_IPP

#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>
#include <dds/network_utils/multiservice.hpp>

#include <nlohmann/json.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename Data, typename Solution>
AsyncMultiServiceServer<Data, Solution>::AsyncMultiServiceServer(
        const types::AmlipIdDataType& own_id,
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler)
    : processing_(false)
    , own_id_(own_id)
    , topic_(topic)
{
    nlohmann::json property_value;

    property_value["Internal"] = "AsyncMultiServiceServer Node";

    property_value["Entity"] = "Reader";
    property_value["Topic"] = utils::multiservice_topic_mangling(topic,
                    utils::MultiServiceTopicType::request_availability);
    eprosima::fastdds::dds::DataReaderQos qos_request_availability_reader_ = default_request_availability_reader_qos_();
    qos_request_availability_reader_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);

    request_availability_reader_ = std::make_shared<Reader<types::MsRequestDataType>>(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::request_availability),
        dds_handler,
        qos_request_availability_reader_); // REQUEST_AVAILABILITY

    property_value["Entity"] = "DirectWriter";
    property_value["Topic"] = utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::reply_available);
    eprosima::fastdds::dds::DataWriterQos qos_reply_available_writer_ =
            DirectWriter<types::MsReferenceDataType>::default_directwriter_qos();
    qos_reply_available_writer_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);


    reply_available_writer_ = std::make_shared<DirectWriter<types::MsReferenceDataType>>(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::reply_available),
        dds_handler,
        qos_reply_available_writer_); // REPLY_AVAILABLE

    property_value["Entity"] = "Reader";
    property_value["Topic"] = utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_target);
    eprosima::fastdds::dds::DataReaderQos qos_task_target_reader_ = default_task_target_reader_qos_();
    qos_task_target_reader_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);

    task_target_reader_ = std::make_shared<Reader<types::MsReferenceDataType>>(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_target),
        dds_handler,
        qos_task_target_reader_); // TASK_TARGET

    property_value["Entity"] = "TargetedReader";
    property_value["Topic"] = utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_data);
    eprosima::fastdds::dds::DataReaderQos qos_task_data_reader_ =
            TargetedReader<types::MsDataType<Data>>::default_targetedreader_qos();
    qos_task_data_reader_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);

    task_data_reader_ = std::make_shared<TargetedReader<types::MsDataType<Data>>>(
        own_id,
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_data),
        dds_handler,
        qos_task_data_reader_); // TASK_DATA

    property_value["Entity"] = "DirectWriter";
    property_value["Topic"] = utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_solution);
    eprosima::fastdds::dds::DataWriterQos qos_task_solution_writer_ =
            DirectWriter<types::MsDataType<Solution>>::default_directwriter_qos();
    qos_task_solution_writer_.properties().properties().emplace_back("fastdds.application.metadata",
            property_value.dump(), true);

    task_solution_writer_ = std::make_shared<DirectWriter<types::MsDataType<Solution>>>(
        utils::multiservice_topic_mangling(topic, utils::MultiServiceTopicType::task_solution),
        dds_handler,
        qos_task_solution_writer_); // TASK_SOLUTION

    logDebug(
        AMLIPCPP_DDS_MSASYNCSERVER,
        "New Async MultiService Server " << *this << " created.");
}

template <typename Data, typename Solution>
AsyncMultiServiceServer<Data, Solution>::~AsyncMultiServiceServer()
{
    stop();

    logDebug(
        AMLIPCPP_DDS_MSASYNCSERVER,
        "Async MultiService Server " << *this << " destroyed.");
}

template <typename Data, typename Solution>
void AsyncMultiServiceServer<Data, Solution>::run(
        std::shared_ptr<TaskListener<Data, Solution>> processing_listener)
{
    // If it is not yet processing data, start thread to process it. Otherwise fails.
    if (!processing_.exchange(true))
    {
        logDebug(
            AMLIPCPP_DDS_MSASYNCSERVER,
            "Starting processing data in " << *this << " .");

        processing_thread_ = std::thread(
            &AsyncMultiServiceServer<Data, Solution>::processing_routine_async_,
            this,
            processing_listener);
    }
    else
    {
        throw eprosima::utils::InconsistencyException(
                  STR_ENTRY << "AsyncMultiServiceServer node " << own_id_ << " is already processing data.");
    }
}

template <typename Data, typename Solution>
void AsyncMultiServiceServer<Data, Solution>::stop()
{
    // If it is processing data, set processing as false and wait for process to finish
    if (processing_.exchange(false))
    {
        logDebug(
            AMLIPCPP_DDS_MSASYNCSERVER,
            "Stopping processing data " << *this << " .");

        processing_thread_.join();
    }
}

template <typename Data, typename Solution>
void AsyncMultiServiceServer<Data, Solution>::processing_routine_async_(
        std::shared_ptr<TaskListener<Data, Solution>> processing_listener)
{
    logDebug(AMLIPCPP_DDS_MSASYNCSERVER, "Processing tasks in : " << *this << ".");

    while (processing_)
    {

        types::MsReferenceDataType task_target;

        while (processing_)
        {
            // WAIT FOR REQUEST AVAILABILITY
            // Wait a minimum timeout
            auto reason = request_availability_reader_->wait_data_available(WAIT_AVAILABILITY_TIMEOUT_);
            if (reason != eprosima::utils::event::AwakeReason::condition_met)
            {
                continue;
            }

            // read request
            types::MsRequestDataType request = request_availability_reader_->read();

            // If this task is already known to have been targeted, remove it and turn back to main loop
            auto it = already_processed_requests_.find(request);
            if (it != already_processed_requests_.end())
            {
                already_processed_requests_.erase(it);
                continue;
            }

            logDebug(AMLIPCPP_DDS_MSASYNCSERVER, "Request received: " << request << ". Sending available reply.");

            // ANSWER AVAILABLE
            // Create data
            types::MsReferenceDataType reference(
                request.client_id(),
                request.task_id(),
                own_id_);

            // Write data
            reply_available_writer_->write(reference.client_id(), reference);


            // WAIT FOR TASK TARGET
            // NOTE: this should not exit until the target has been received and answered (if required)
            // So no breaks here or processing checks
            while (processing_)
            {
                // Wait for data to arrive
                auto reason = task_target_reader_->wait_data_available(WAIT_AVAILABILITY_TIMEOUT_);
                if (reason != eprosima::utils::event::AwakeReason::condition_met)
                {
                    continue;
                }

                // Read data
                task_target = task_target_reader_->read();

                // Add it as already processed requests so it is not possible to wait for it again
                already_processed_requests_.insert(task_target.request());

                // Check if it is the reference we are expecting
                if (task_target.client_id() != reference.client_id() ||
                        task_target.task_id() != reference.task_id())
                {
                    // If not, continue waiting
                    logDebug(AMLIPCPP_DDS_MSASYNCSERVER,
                            "Request received but not valid: " << task_target << ". Keep waiting.");
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
                    AMLIPCPP_DDS_MSASYNCSERVER,
                    "Task: " << request << " lost due to other server has been taken. Start over.");
            }
        }

        // If exit because it should stop processing, the target is not done
        if (task_target.server_id() != own_id_)
        {
            break;
        }

        // From here, task_target has the reference for the task this must solve
        logDebug(AMLIPCPP_MULTISERVICE_SERVER,
                "Server " << own_id_ << " processing task: " << task_target.task_id() <<
                " from client: " << task_target.client_id() << ".");

        // WAIT FOR TASK DATA
        while (processing_)
        {
            auto reason = task_data_reader_->wait_data_available(WAIT_AVAILABILITY_TIMEOUT_);
            if (reason != eprosima::utils::event::AwakeReason::condition_met)
            {
                continue;
            }

            types::MsDataType<Data> ms_data = task_data_reader_->read();

            // PROCESS DATA AND SEND SOLUTION

            // Process solution from callback
            auto solution = processing_listener->process_task(
                std::make_unique<Data>(ms_data.data()),
                task_target.task_id(),
                task_target.client_id(),
                task_target.server_id()
                );

            // Create solution data
            types::MsDataType<Solution> ms_solution(
                types::MsReferenceDataType(task_target),
                std::move(solution));

            // Send solution
            task_solution_writer_->write(task_target.client_id(), ms_solution);

            break;
        }
    }
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataReaderQos AsyncMultiServiceServer<Data,
        Solution>::default_request_availability_reader_qos_()
{
    eprosima::fastdds::dds::DataReaderQos qos = utils::default_datareader_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;
    // TODO this should change to Keep last 1 when keys are implemented

    return qos;
}

template <typename Data, typename Solution>
eprosima::fastdds::dds::DataReaderQos AsyncMultiServiceServer<Data, Solution>::default_task_target_reader_qos_()
{
    return default_request_availability_reader_qos_();
}

template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const AsyncMultiServiceServer<Data, Solution>& obj)
{
    os << "ASYNC_MSSERVER{" << obj.own_id_ << ";" << obj.topic_ << "}";
    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_IMPL_ASYNCMULTISERVICESERVER_IPP */
