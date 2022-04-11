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

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Reader<T>::Reader(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataReader> data_reader)
    : topic_(topic)
    , data_reader_(data_reader)
    , stop_(false)
    , data_available_count_(0)
{
}

template <typename T>
Reader<T>::~Reader()
{
    stop();
}

template <typename T>
void Reader<T>::wait_data_available(uint32_t timeout /* = 0 */)
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    data_available_condition_variable_.wait(
        lock,
        [this]
        {
            // Exit if number of events is bigger than expected n
            // or if callback is no longer set
            return data_available_count_ > 0 || stop_.load();
        });
}

template <typename T>
void Reader<T>::stop()
{
    {
        std::lock_guard<std::mutex> lock(data_available_mutex_);
        stop_.store(true);
    }
    data_available_condition_variable_.notify_one();
}

template <typename T>
std::shared_ptr<T> Reader<T>::read()
{
    // std::lock_guard<std::mutex> lock(data_available_mutex_);

    eprosima::fastdds::dds::SampleInfo info;
    std::shared_ptr<T> data = std::make_shared<T>();

    eprosima::fastrtps::types::ReturnCode_t return_code = data_reader_->take_next_sample(data.get(), &info);
    if (return_code == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
    {
        std::lock_guard<std::mutex> lock(data_available_mutex_);
        data_available_count_--;
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
bool Reader<T>::is_data_available()
{
    std::lock_guard<std::mutex> lock(data_available_mutex_);
    return data_available_count_ > 0 && !stop_.load();
}

template <typename T>
void Reader<T>::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{
    {
        std::lock_guard<std::mutex> lock(data_available_mutex_);
        data_available_count_ = reader->get_unread_count();
    }
    data_available_condition_variable_.notify_one();
}

template <typename T>
void Reader<T>::on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    // if (info.current_count_change > 0)
    // {
    //     std::cout << "Subscriber matched." << std::endl;
    // }
    // else if (info.current_count_change < 0)
    // {
    //     std::cout << "Subscriber unmatched." << std::endl;
    // }
}

template <typename T>
eprosima::fastdds::dds::DataReaderQos Reader<T>::default_datareader_qos()
{
    return eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP */
