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
 * @file Reader.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_READER_HPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_READER_HPP

#include <atomic>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <ddsrouter_event/wait/BooleanWaitHandler.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Reader Listener with callback to notify when readers match.
 */
class ReaderListener : public eprosima::fastdds::dds::DataReaderListener
{
public:

    // Default constructor and destructor

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

    void set_callback(std::function<void()> on_data_available_callback) noexcept;

protected:

    std::atomic<bool> callback_set_;

    std::function<void()> on_data_available_callback_;
};

/**
 * @brief Class that allows to subscribe to DDS topic and listen messages
 *
 * @tparam T
 */
template <typename T>
class Reader
{
public:

    Reader(
        const std::string& topic,
        std::unique_ptr<ReaderListener> listener,
        std::shared_ptr<eprosima::fastdds::dds::DataReader> datareader);

    virtual ~Reader();

    void wait_data_available(uint32_t timeout = 0);

    bool is_data_available();

    T read();

    static eprosima::fastdds::dds::DataReaderQos default_datareader_qos();

protected:

    void new_data_available_();

    //! Name of the topic this Reader subscribes to
    std::string topic_;

    //! DDS DataReader reference
    std::shared_ptr<eprosima::fastdds::dds::DataReader> data_reader_;

    //! Reader Listener
    std::unique_ptr<ReaderListener> listener_;

    //! Waiter variable to wait for a data to be available
    ddsrouter::event::BooleanWaitHandler reader_data_waiter_;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Reader.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_READER_HPP */
