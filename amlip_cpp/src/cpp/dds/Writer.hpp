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

#ifndef AMLIPCPP__SRC_CPP_DDS_WRITER_HPP
#define AMLIPCPP__SRC_CPP_DDS_WRITER_HPP

#include <atomic>

#include <fastrtps/rtps/writer/WriterListener.h>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include <cpp_utils/wait/BooleanWaitHandler.hpp>
#include <cpp_utils/memory/owner_ptr.hpp>

#include <dds/DdsHandler.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Writer Listener with callback to notify when readers match.
 */
class WriterListener : public eprosima::fastdds::dds::DataWriterListener
{
public:

    // Default constructor
    WriterListener();

    //! Default destructor
    virtual ~WriterListener();

    /**
     * @brief Listener callback when a DataReader has been matched or unmatched
     *
     * @param writer internal DataWriter
     * @param info Match information
     */
    void on_publication_matched(
            eprosima::fastdds::dds::DataWriter* writer,
            const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

    //! Number of readers currently matched
    uint32_t readers_matched() const noexcept;

    /**
     * @brief Stop this thread for the Writer to be matched
     *
     * This method will block until the Writer is matched or the \c stop() method is called.
     *
     * @param timeout_ms maximum wait time in milliseconds (0 = no wait)
     *
     * @return Reason for the awaken.
     */
    eprosima::utils::event::AwakeReason wait_match(
            const eprosima::utils::Duration_ms& timeout = 0);

protected:

    //! Increment the number of readers matched. (Awake waiting threads).
    void increase_match_() noexcept;

    //! Decrement the number of readers matched.
    void decrease_match_() noexcept;

    //! Number of readers matched
    std::atomic<uint32_t> matched_readers_;

    //! Waiter variable to wait for a reader to be matched
    eprosima::utils::event::BooleanWaitHandler writer_match_waiter_;
};

/**
 * @brief Class that allows to publish messages
 *
 * @tparam T
 */
template <typename T>
class Writer : public WriterListener
{
public:

    /**
     * @brief Construct a new Writer object using a DDS DataWriter already created.
     *
     * The DataWriter
     *
     * @pre \c datawriter must be a valid pointer to a DataWriter, must be disabled and without listener.
     *
     * @param topic DDS topic name in which the data this Writer publish will be published
     * @param datawriter disabled datawriter reference without listener set
     *
     * @remarks This constructor is protected because this object should be created from Participant (factory).
     */
    Writer(
            const std::string& topic,
            eprosima::utils::LesseePtr<DdsHandler> dds_handler,
            eprosima::fastdds::dds::DataWriterQos qos = Writer::default_datawriter_qos());

    //! Default destructor, stop listener before destruction
    virtual ~Writer();

    /**
     * @brief Publish new data in this Writer Topic
     *
     * This write will be done asynchronously and the data will be copied, so \c data could be destroyed afterwards.
     *
     * @param data new data to write
     *
     * @note the parameter should be const, but could not be as fastdds write requires a non const (because of reasons)
     */
    eprosima::fastrtps::types::ReturnCode_t publish(
            T& data);

    /**
     * @brief Return default QoS for a DataWriter
     *
     * Default Writer QoS is:
     * - Allocation: PREALLOCATED_WITH_REALLOC
     * The rest of values are left as Fast-DDS default
     *
     * @return eprosima::fastdds::dds::DataWriterQos
     */
    static eprosima::fastdds::dds::DataWriterQos default_datawriter_qos();

protected:

    //! Name of the topic this Writer publishes
    std::string topic_;

    //! DDS DataWriter reference
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataWriter> datawriter_;

    // Allow operator << to access private members
    template <typename U>
    friend std::ostream& operator <<(
            std::ostream& os,
            const Writer<U>& obj);
};

constexpr const unsigned int Wait_for_acknowledgments_time_s = 1;
constexpr const unsigned int Wait_before_destroying_time_ms = 200;

//! \c Writer to stream serializator
template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const Writer<T>& obj);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/Writer.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_WRITER_HPP */
