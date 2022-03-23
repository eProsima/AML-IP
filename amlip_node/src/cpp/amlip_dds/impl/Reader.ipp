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
        std::shared_ptr<eprosima::fastdds::dds::DataReader> reader)
    : topic_(topic)
    , data_reader_(reader)
    , stop_(false)
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
        [n, this]
        {
            // Exit if number of events is bigger than expected n
            // or if callback is no longer set
            return data_available_count_ > 0 || stop_.load();
        });
}

template <typename T>
void Reader<T>::stop()
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    stop_.store(false);
}

template <typename T>
T Reader<T>::read(void* data)
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);

    eprosima::fastdds::dds::SampleInfo info;
    T data;

    eprosima::fastrtps::types::ReturnCode_t return_code = reader->take_next_sample(&data, &info);
    if (return_code == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK)
    {
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
    return stop_.load();
}

template <typename T>
void Reader<T>::on_data_available(
        eprosima::fastdds::dds::DataReader* reader) override
{
    std::unique_lock<std::mutex> lock(data_available_mutex_);
    data_available_count_++;
    data_available_condition_variable_.notify_one();
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Reader<T>::default_datareader_qos_() const
{
    return eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP */
