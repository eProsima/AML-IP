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

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_DIRECTWRITER_HPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_DIRECTWRITER_HPP

#include <atomic>

#include <fastrtps/rtps/writer/WriterListener.h>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include <ddsrouter_event/wait/BooleanWaitHandler.hpp>
#include <ddsrouter_utils/memory/OwnerPtr.hpp>

#include <dds/DdsHandler.hpp>
#include <types/AmlipId.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Class that allows to publish messages
 *
 * @tparam T
 */
template <typename T>
class DirectWriter
{
public:

    /**
     * @brief Construct a new DirectWriter object using a DDS DataWriter already created.
     *
     * The DataWriter
     *
     * @pre \c datawriter must be a valid pointer to a DataWriter, must be disabled and without listener.
     *
     * @param topic DDS topic name in which the data this DirectWriter publish will be published
     * @param datawriter disabled datawriter reference without listener set
     *
     * @remarks This constructor is protected because this object should be created from Participant (factory).
     */
    DirectWriter(
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataWriterQos qos = DirectWriter::default_directwriter_qos());

    //! Default destructor, stop listener before destruction
    virtual ~DirectWriter();

    // TODO add a discovery database and check if target exists

    /**
     * TODO
     */
    eprosima::fastrtps::types::ReturnCode_t write(
        const types::AmlipId& target_id,
        T& data);

    /**
     * @brief Return default QoS for a DataWriter
     *
     * Default DirectWriter QoS is:
     * - Allocation: PREALLOCATED_WITH_REALLOC
     * The rest of values are left as Fast-DDS default
     *
     * @return eprosima::fastdds::dds::DataWriterQos
     */
    static eprosima::fastdds::dds::DataWriterQos default_directwriter_qos();

protected:

    ddsrouter::utils::LesseePtr<eprosima::fastdds::dds::DataWriter> get_target_datawriter_(
        types::AmlipId target_id);

    //! Name of the topic this DirectWriter publishes
    std::string topic_;

    ddsrouter::utils::LesseePtr<DdsHandler> dds_handler_;

    eprosima::fastdds::dds::DataWriterQos qos_;

    //! DDS DataWriter reference
    std::map<types::AmlipId, ddsrouter::utils::LesseePtr<eprosima::fastdds::dds::DataWriter>> writers_;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/DirectWriter.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_DIRECTWRITER_HPP */
