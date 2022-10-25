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

#ifndef AMLIPCPP__SRC_CPP_DDS_IMPL_READER_IPP
#define AMLIPCPP__SRC_CPP_DDS_IMPL_READER_IPP

#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>

#include <dds/network_utils/dds_qos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Reader<T>::Reader(
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataReaderQos qos /* = Reader::default_datareader_qos() */)
    : topic_(topic)
{
    auto dds_handler_locked = dds_handler.lock_with_exception();

    datareader_ = dds_handler_locked->create_datareader<T>(
        topic_,
        qos);

    datareader_->set_listener(this);

    logDebug(
        AMLIPCPP_DDS_READER,
        "Reader created: " << *this << ".");
}

template <typename T>
Reader<T>::~Reader()
{
    // Stop every waiting thread
    reader_data_waiter_.blocking_disable();

    logDebug(
        AMLIPCPP_DDS_READER,
        "Destroying Reader: " << *this << ".");

    auto guarded_ptr = datareader_.lock();
    if (guarded_ptr)
    {
        guarded_ptr->set_listener(nullptr);
    }
}

template <typename T>
eprosima::utils::event::AwakeReason Reader<T>::wait_data_available(uint32_t timeout /* = 0 */)
{
    return reader_data_waiter_.wait();
}

template <typename T>
void Reader<T>::awake_waiting_threads()
{
    reader_data_waiter_.stop_and_continue();
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

    logDebug(
        AMLIPCPP_DDS_READER,
        "Reading message from: " << *this << ".");

    // TODO: This creates a new data, ergo it is copying the data arrived. Check and refactor this

    eprosima::fastrtps::types::ReturnCode_t return_code = datareader_locked_->take_next_sample(&data, &info);
    if (return_code == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
    {
        // Set number of data still available
        if (datareader_locked_->get_unread_count() <= 0)
        {
            // TODO: there is a race condition here as get_unread_count is not guarded by any mutex here
            // and so a call to is_data_available could be skipped here.
            reader_data_waiter_.close();
        }

        logDebug(
            AMLIPCPP_DDS_READER,
            "Reader: " << *this << " received message: " << data << ".");

        // Return data (this does a move)
        return data;
    }
    else
    {
        // TODO
        throw eprosima::utils::InconsistencyException(
                  "Try to read data from a reader that does not have it.");
    }
}

template <typename T>
eprosima::fastdds::dds::DataReaderQos Reader<T>::default_datareader_qos()
{
    return utils::default_datareader_qos();
}

template <typename T>
void Reader<T>::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{
    logDebug(AMLIP_READER, "Reader " << *this << " has received a data.");

    reader_data_waiter_.open();
}

template <typename T>
void Reader<T>::on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change > 0)
    {
        logDebug(
            AMLIP_READER, "Reader " << *this << " matched with Writer: " << info.last_publication_handle << ".");
    }
    else if (info.current_count_change < 0)
    {
        logDebug(
            AMLIP_READER, "Reader " << *this << " unmatched with Writer: "  << info.last_publication_handle << ".");
    }
}

template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const Reader<T>& obj)
{
    os << "READER{";
    os << obj.topic_ << ";";
    os << obj.datareader_->guid();
    os << "}";

    return os;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_IMPL_READER_IPP */
