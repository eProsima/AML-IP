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
 * @file MultiServiceClient.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_MULTISERVICECLIENT_HPP
#define AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_MULTISERVICECLIENT_HPP

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

/**
 * TODO
 */
template <typename Data, typename Solution>
class MultiServiceClient
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    MultiServiceClient(
            const types::AmlipIdDataType& own_id,
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler);

    ~MultiServiceClient();

    /**
     * @brief
     *
     * @param data
     * @return Solution
     *
     * @warning This method is thought to use MS in only one thread. Multithreading synchronization is not implemented.
     * Thus, using multiple threads will cause desynchronization of messages received and locks.
     */
    Solution send_request_sync(
            const Data& data);

protected:

    types::TaskId new_task_id_();

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

};

//! \c MultiServiceClient to stream serializator
template <typename Data, typename Solution>
std::ostream& operator <<(
        std::ostream& os,
        const MultiServiceClient<Data, Solution>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/multiservice/impl/MultiServiceClient.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_MULTISERVICE_MULTISERVICECLIENT_HPP */
