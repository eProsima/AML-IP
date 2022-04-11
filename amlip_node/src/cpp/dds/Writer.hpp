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
 * @file Writer.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPDDS_WRITER_HPP
#define AMLIP__SRC_CPP_AMLIPDDS_WRITER_HPP

#include <atomic>
#include <condition_variable>

#include <fastrtps/rtps/writer/WriterListener.h>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
class Writer : public eprosima::fastdds::dds::DataWriterListener
{
public:

    // TODO
    Writer(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataWriter> datawriter);

    virtual ~Writer();

    void wait_writer_matched();

    void stop();

    bool stopped();

    void write(T& data);

    void on_publication_matched(
            eprosima::fastdds::dds::DataWriter* writer,
            const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

    static eprosima::fastdds::dds::DataWriterQos default_datawriter_qos();

protected:

    std::string topic_;

    std::atomic<bool> stop_;

    std::atomic<bool> writer_matched_;

    std::condition_variable writer_matched_condition_variable_;

    std::mutex writer_matched_mutex_;

    std::shared_ptr<eprosima::fastdds::dds::DataWriter> data_writer_;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Writer.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPDDS_WRITER_HPP */
