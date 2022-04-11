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
 * @file Writer.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Writer<T>::Writer(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataWriter> datawriter)
    : topic_(topic)
    , data_writer_(datawriter)
    , stop_(false)
    , writer_matched_(false)
{
}

template <typename T>
Writer<T>::~Writer()
{
}

template <typename T>
void Writer<T>::wait_writer_matched()
{
    std::unique_lock<std::mutex> lock(writer_matched_mutex_);
    writer_matched_condition_variable_.wait(
        lock,
        [this]
        {
            return writer_matched_.load() || stop_.load();
        });
    writer_matched_.store(false);
}

template <typename T>
void Writer<T>::stop()
{
    {
        std::lock_guard<std::mutex> lock(writer_matched_mutex_);
        stop_.store(true);
    }
    writer_matched_condition_variable_.notify_one();
}

template <typename T>
bool Writer<T>::stopped()
{
    std::lock_guard<std::mutex> lock(writer_matched_mutex_);
    return stop_.load();
}

template <typename T>
void Writer<T>::write(T& data)
{
    data_writer_->write(&data);
}

template <typename T>
void Writer<T>::on_publication_matched(
        eprosima::fastdds::dds::DataWriter* writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    // if (info.current_count_change > 0)
    // {
    //     std::cout << "Publisher matched." << std::endl;
    //     {
    //         std::lock_guard<std::mutex> lock(writer_matched_mutex_);
    //         writer_matched_.store(true);
    //     }
    //     writer_matched_condition_variable_.notify_one();
    // }
    // else if (info.current_count_change < 0)
    // {
    //     std::cout << "Publisher unmatched." << std::endl;
    // }
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Writer<T>::default_datawriter_qos()
{
    return eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP */
