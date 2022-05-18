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
 * @file multiservice.cpp
 */

#include <dds/network_utils/multiservice.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

std::string multiservice_topic_mangling(
        const std::string& actual_topic_name,
        const MultiServiceTopicType& internal_topic_type)
{
    std::string mangled_topic_name = actual_topic_name;
    switch (internal_topic_type)
    {
        case MultiServiceTopicType::REQUEST_AVAILABILITY:
            mangled_topic_name += "_request_availability";
            break;
        case MultiServiceTopicType::REPLY_AVAILABLE:
            mangled_topic_name += "_reply_available";
            break;
        case MultiServiceTopicType::TASK_TARGET:
            mangled_topic_name += "_task_target";
            break;
        case MultiServiceTopicType::TASK_DATA:
            mangled_topic_name += "_task_data";
            break;
        case MultiServiceTopicType::TASK_SOLUTION:
            mangled_topic_name += "_task_solution";
            break;
        default:
            break;
    }
    return mangled_topic_name;
}

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
