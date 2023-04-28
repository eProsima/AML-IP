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
 * @file topic.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NETWORK_TOPIC_HPP
#define AMLIPCPP__SRC_CPP_NETWORK_TOPIC_HPP

#include <string>

#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

/////////////////
// STATUS TOPIC
constexpr const char* STATUS_TOPIC_NAME = "status";

eprosima::fastdds::dds::DataWriterQos status_writer_qos() noexcept;

eprosima::fastdds::dds::DataReaderQos status_reader_qos() noexcept;

/////////////////
// JOB TOPIC
constexpr const char* JOB_TOPIC_NAME = "job";

/////////////////
// MODEL TOPIC
constexpr const char* MODEL_TOPIC_NAME = "model";

eprosima::fastdds::dds::DataWriterQos model_writer_qos() noexcept;

eprosima::fastdds::dds::DataReaderQos model_reader_qos() noexcept;

/////////////////
// MANGLING
constexpr const char* TYPE_NAME_MANGLING = "amlip::";

constexpr const char* TOPIC_NAME_MANGLING = "amlip/";

std::string type_name_mangling(
        const std::string& type_name) noexcept;

std::string topic_name_mangling(
        const std::string& type_name) noexcept;

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NETWORK_TOPIC_HPP */
