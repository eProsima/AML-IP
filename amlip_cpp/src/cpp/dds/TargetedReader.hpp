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
 * @file TargetedReader.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_TARGETEDREADER_HPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_TARGETEDREADER_HPP

#include <atomic>

#include <dds/Reader.hpp>
#include <types/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief Class that allows to publish messages
 *
 * @tparam T
 */
template <typename T>
class TargetedReader : public Reader<T>
{
public:

    TargetedReader(
        const types::AmlipIdDataType own_id,
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler,
        eprosima::fastdds::dds::DataReaderQos qos = TargetedReader<T>::default_targetedreader_qos());

    static eprosima::fastdds::dds::DataReaderQos default_targetedreader_qos();

};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/TargetedReader.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_TARGETEDREADER_HPP */
