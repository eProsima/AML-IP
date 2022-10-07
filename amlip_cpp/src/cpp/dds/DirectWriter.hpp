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
 * @file DirectWriter.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_DIRECTWRITER_HPP
#define AMLIPCPP__SRC_CPP_DDS_DIRECTWRITER_HPP

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
#include <dds/Writer.hpp>
#include <types/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Class that allows to write messages to a specific receiver ( \c TargetedReader ).
 *
 * Direct Writer is implemented by using a specific topic for each receiver in the network.
 * For each TargetedReader, the topic used is mangled so only that TargetedReader is listening in that specific topic.
 * This class creates Writers each time a new receiver is needed.
 *
 * @tparam T
 */
template <typename T>
class DirectWriter
{
public:

    /**
     * @brief Construct a new DirectWriter object
     *
     * This object will handle a copy of the DdsHandler (as a lessee) so it can create new Writers for each
     * receiver needed.
     *
     * TODO
     */
    DirectWriter(
        const std::string& topic,
        eprosima::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataWriterQos qos = DirectWriter::default_directwriter_qos());

    //! Default destructor, stop listener before destruction
    virtual ~DirectWriter();

    // TODO add a discovery database and check if target exists

    /**
     * TODO
     */
    eprosima::fastrtps::types::ReturnCode_t write(
        const types::AmlipIdDataType& target_id,
        T& data);

    /**
     * @brief Stop this thread for the Writer to be matched with specific reader
     *
     * This method will block until the Writer is matched or timeout is reached.
     *
     * @param timeout_ms maximum wait time in milliseconds (0 = no wait)
     *
     * @return Reason for the awakening.
     */
    eprosima::utils::event::AwakeReason wait_match(
        const types::AmlipIdDataType& target_id,
        const eprosima::utils::Duration_ms &timeout = 0);

    /**
     * @brief Return default QoS for a DataWriter
     *
     * Default DirectWriter QoS is:
     * - Allocation: PREALLOCATED_WITH_REALLOC
     * - Durability: VOLATILE
     * - Reliability: RELIABLE
     * - History: KEEP_ALL
     * The rest of values are left as Fast-DDS default
     *
     * @return eprosima::fastdds::dds::DataWriterQos
     */
    static eprosima::fastdds::dds::DataWriterQos default_directwriter_qos();

protected:

    std::shared_ptr<Writer<T>> get_target_writer_(
        types::AmlipIdDataType target_id);

    //! Name of the topic in which this \c DirectWriter publishes
    std::string topic_;

    eprosima::utils::LesseePtr<DdsHandler> dds_handler_;

    eprosima::fastdds::dds::DataWriterQos qos_;

    /**
     * @brief
     *
     * @note it is a shared ptr because it is not distributed outside this class.
     * If in the future it must be used outside the class, consider using OwnerPtr
     * @note it could also be developed as having Writers instead of dataWriters and Listeners
     */
    std::map<types::AmlipIdDataType, std::shared_ptr<Writer<T>>> writers_;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/DirectWriter.ipp>

#endif /* AMLIPCPP__SRC_CPP_DDS_DIRECTWRITER_HPP */
