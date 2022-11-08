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

#ifndef AMLIPCPP__SRC_CPP_DDS_READER_HPP
#define AMLIPCPP__SRC_CPP_DDS_READER_HPP

#include <atomic>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include <cpp_utils/wait/BooleanWaitHandler.hpp>
#include <cpp_utils/memory/owner_ptr.hpp>

#include <dds/DdsHandler.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Class that allows to subscribe to DDS topic and listen messages
 *
 * @tparam T
 */
template <typename T>
class Reader : public eprosima::fastdds::dds::DataReaderListener
{
public:

    Reader(
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler,
            eprosima::fastdds::dds::DataReaderQos qos = Reader::default_datareader_qos());

    virtual ~Reader();

    eprosima::utils::event::AwakeReason wait_data_available(
            uint32_t timeout = 0);

    void awake_waiting_threads();

    bool is_data_available();

    T read();

    static eprosima::fastdds::dds::DataReaderQos default_datareader_qos();

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    void on_subscription_matched(
            eprosima::fastdds::dds::DataReader* reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

protected:

    //! Name of the topic this Reader subscribes to
    std::string topic_;

    //! DDS DataReader reference
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataReader> datareader_;

    //! Waiter variable to wait for a data to be available
    eprosima::utils::event::BooleanWaitHandler reader_data_waiter_;

    // Allow operator << to access private members
    template <typename U>
    friend std::ostream& operator <<(
            std::ostream& os,
            const Reader<U>& obj);
};

//! \c Reader to stream serializator
template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const Reader<T>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Reader.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_READER_HPP */
