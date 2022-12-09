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
 * @file AsyncMultiServiceClient.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICECLIENT_HPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICECLIENT_HPP

#include <cpp_utils/wait/DBQueueWaitHandler.hpp>
#include <cpp_utils/wait/BooleanWaitHandler.hpp>
#include <cpp_utils/wait/CounterWaitHandler.hpp>
#include <cpp_utils/types/Atomicable.hpp>

#include <dds/DdsHandler.hpp>
#include <dds/DirectWriter.hpp>
#include <dds/Reader.hpp>
#include <dds/TargetedReader.hpp>
#include <dds/Writer.hpp>
#include <types/multiservice/MsDataType.hpp>
#include <types/multiservice/MsRequestDataType.hpp>
#include <types/multiservice/MsReferenceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

constexpr const unsigned int ASYNC_MULTISERVICE_N_THREADS = 3;

template <typename Solution>
class SolutionListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~SolutionListener() = default;

    /**
     * @brief Method that will be called with the Job message received to calculate an answer.
     *
     * @param job new Job message received.
     * @param task_id Id of the Task received.
     * @param client_id Id of the Client that sent this job.
     *
     * @return Solution to the \c job .
     */
    virtual void solution_received(
        std::unique_ptr<Solution> solution,
        const types::TaskId& task_id,
        const eprosima::amlip::types::AmlipIdDataType& client_id,
        const types::AmlipIdDataType& server_id) = 0;
};

/**
 * TODO
 */
template <typename Data, typename Solution>
class AsyncMultiServiceClient
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    AsyncMultiServiceClient(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    ~AsyncMultiServiceClient();

    void run(std::shared_ptr<SolutionListener<Solution>> solution_listener);

    void stop();

    /**
     * @brief
     *
     * @param data
     * @return Solution
     *
     * @warning This method is thought to use MS in only one thread. Multithreading synchronization is not implemented.
     * Thus, using multiple threads will cause desynchronization of messages received and locks.
     */
    types::TaskId send_request_async(
            std::shared_ptr<Data> data);

protected:

    types::TaskId new_task_id_();

    void send_request_async_routine_();

    void read_solution_();

    void read_replies_();

    static eprosima::fastdds::dds::DataWriterQos default_request_availability_writer_qos_();
    static eprosima::fastdds::dds::DataWriterQos default_task_target_writer_qos_();

    Writer<types::MsRequestDataType> request_availability_writer_;

    TargetedReader<types::MsReferenceDataType> reply_available_reader_;

    Writer<types::MsReferenceDataType> task_target_writer_;

    DirectWriter<types::MsDataType<Data>> task_data_writer_;

    TargetedReader<types::MsDataType<Solution>> task_solution_reader_;

    types::AmlipIdDataType own_id_;

    std::string topic_;

    std::atomic<types::TaskId> last_task_id_used_;

    std::unique_ptr<eprosima::utils::event::ConsumerWaitHandler<
        std::pair<std::shared_ptr<Data>, types::TaskId>>> data_to_send_;

    // TODO: refactor this for a thread pool
    std::thread send_task_request_thread_;

    std::shared_ptr<SolutionListener<Solution>> solution_listener_;

    std::atomic<bool> running_;

    ///////////////////
    // REPLY READER

    eprosima::utils::event::CounterWaitHandler waiter_for_request_;

    std::thread reply_reading_thread_;

    std::unique_ptr<eprosima::utils::event::ConsumerWaitHandler<
        types::MsReferenceDataType>> server_to_send_;

    std::set<types::TaskId> already_chosen_tasks_;

    ///////////////////
    // SOLUTION READER

    std::thread solution_reading_thread_;

    std::atomic<bool> should_stop_;

    template <typename D, typename S>
    friend std::ostream& operator <<(std::ostream& , const AsyncMultiServiceClient<D, S>& );
};

//! \c AsyncMultiServiceClient to stream serializator
template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const AsyncMultiServiceClient<Data, Solution>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/multiservice/impl/AsyncMultiServiceClient.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICECLIENT_HPP */
