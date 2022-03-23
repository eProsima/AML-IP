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

#ifndef AMLIP__SRC_CPP_AMLIPDDS_READER_HPP
#define AMLIP__SRC_CPP_AMLIPDDS_READER_HPP

#include <fastrtps/rtps/reader/ReaderListener.h>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
class Reader : public eprosima::fastdds::dds::DataReaderListener
{
public:

    Reader(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataReader> reader);

    virtual ~Reader();

    void wait_data_available(uint32_t timeout = 0);

    void stop();

    T read(void* data);

    bool is_data_available();

    void on_data_available(
            eprosima::fastdds::dds::DataReader* reader) override;

    static eprosima::fastdds::dds::DataWriterQos default_datareader_qos_() const;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <impl/Reader.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPDDS_READER_HPP */
