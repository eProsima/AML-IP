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

#ifndef AMLIP__SRC_CPP_AMLIPDDS_DIRECTWRITER_HPP
#define AMLIP__SRC_CPP_AMLIPDDS_DIRECTWRITER_HPP

namespace eprosima {
namespace amlip {
namespace dds {

#include <amlip_dds/Writer.hpp>
#include <amlip_types/AmlipId.hpp>

template <typename T>
class DirectWriter : public eprosima::fastdds::dds::DataWriterListener
{
public:

    // TODO
    DirectWriter(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataWriter> reader);

    virtual ~DirectWriter();

    void write(T data, AmlipId id);

    static eprosima::fastdds::dds::DataWriterQos default_datawriter_qos_() const;
};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_DIRECTWRITER_HPP */
