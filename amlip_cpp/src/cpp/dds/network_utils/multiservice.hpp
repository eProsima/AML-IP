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
 * @file multiservice.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_MULTISERVICE_HPP
#define AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_MULTISERVICE_HPP

#include <string>

#include <cpp_utils/macros/custom_enumeration.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

ENUMERATION_BUILDER(
    MultiServiceTopicType,
    request_availability,
    reply_available,
    task_target,
    task_data,
    task_solution
    );

std::string multiservice_topic_mangling(
        const std::string& actual_topic_name,
        const MultiServiceTopicType& internal_topic_type);

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_MULTISERVICE_HPP */
