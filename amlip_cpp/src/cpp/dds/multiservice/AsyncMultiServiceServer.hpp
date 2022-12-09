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
 * @file AsyncMultiServiceServer.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICESERVER_HPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICESERVER_HPP

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

template <typename Data, typename Solution>
class TaskListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~TaskListener() = default;

    /**
     * @brief Method that will be called with the Job message received to calculate an answer.
     *
     * @param task new Job message received.
     * @param task_id Id of the Task received.
     * @param client_id Id of the Client that sent this task.
     *
     * @return Solution to the \c task .
     */
    virtual Solution process_task (
            std::unique_ptr<Data> task,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id,
            const types::AmlipIdDataType& server_id) = 0;
};

/**
 * TODO
 */
template <typename Data, typename Solution>
class AsyncMultiServiceServer
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    AsyncMultiServiceServer(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    ~AsyncMultiServiceServer();

    void run(std::shared_ptr<TaskListener<Data, Solution>> processing_listener);

    void stop();

protected:

    void processing_routine_async_(std::shared_ptr<TaskListener<Data, Solution>> processing_listener);

    static eprosima::fastdds::dds::DataReaderQos default_request_availability_reader_qos_();
    static eprosima::fastdds::dds::DataReaderQos default_task_target_reader_qos_();

    Reader<types::MsRequestDataType> request_availability_reader_;

    DirectWriter<types::MsReferenceDataType> reply_available_writer_;

    Reader<types::MsReferenceDataType> task_target_reader_;

    TargetedReader<types::MsDataType<Data>> task_data_reader_;

    DirectWriter<types::MsDataType<Solution>> task_solution_writer_;

    std::thread processing_thread_;

    std::atomic<bool> processing_;

    types::AmlipIdDataType own_id_;

    std::string topic_;

    std::set<types::MsRequestDataType> already_processed_requests_;

    template <typename D, typename S>
    friend std::ostream& operator <<(std::ostream& , const AsyncMultiServiceServer<D, S>& );
};

//! \c AsyncMultiServiceServer to stream serializator
template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const AsyncMultiServiceServer<Data, Solution>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/multiservice/impl/AsyncMultiServiceServer.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_ASYNCMULTISERVICESERVER_HPP */
