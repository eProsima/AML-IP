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
 * @file Reader.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Reader<T>::Reader(
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataReaderQos qos /* = Reader::default_datareader_qos() */)
    : topic_(topic)
{
    auto dds_handler_locked = dds_handler.lock_with_exception();

    datareader_ = dds_handler_locked->create_datareader<T>(
            topic_,
            qos,
            this);
}

template <typename T>
Reader<T>::~Reader()
{
    // Stop every waiting thread
    // TODO: this should be blocking or not exist at all
    reader_data_waiter_.disable();
}

template <typename T>
void Reader<T>::wait_data_available(uint32_t timeout /* = 0 */)
{
    reader_data_waiter_.wait();
}

template <typename T>
bool Reader<T>::is_data_available()
{
    return reader_data_waiter_.is_open();
}

template <typename T>
T Reader<T>::read()
{
    auto datareader_locked_ = datareader_.lock_with_exception();

    eprosima::fastdds::dds::SampleInfo info;
    T data;

    // TODO: This creates a new data, ergo it is copying the data arrived. Check and refactor this

    eprosima::fastrtps::types::ReturnCode_t return_code = datareader_locked_->take_next_sample(&data, &info);
    if (return_code == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
    {
        // Set number of data still available
        if (datareader_locked_->get_unread_count() <= 0)
        {
            reader_data_waiter_.close();
        }

        // Return data (this does a move)
        return data;
    }
    else
    {
        // TODO
        throw ddsrouter::utils::InconsistencyException(
            "Try to read data from a reader that does not have it.");
    }
}

template <typename T>
eprosima::fastdds::dds::DataReaderQos Reader<T>::default_datareader_qos()
{
    eprosima::fastdds::dds::DataReaderQos qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;

    // Preallocated with realloc
    qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    return qos;
}

template <typename T>
void Reader<T>::on_data_available(
        eprosima::fastdds::dds::DataReader*)
{
    reader_data_waiter_.open();
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP */
