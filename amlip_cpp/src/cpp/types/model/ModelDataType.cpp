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

/*!
 * @file ModelDataType.cpp
 */

#include <iostream>

#include <amlip_cpp/types/model/ModelDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

ModelDataType::ModelDataType(
        void* data,
        const uint32_t size,
        bool take_ownership /* = false */)
    : GenericDataType(data, size, take_ownership)
{
    // Do nothing
}

ModelDataType::ModelDataType(
        const std::vector<ByteType>& bytes)
    : GenericDataType(bytes)
{
    // Do nothing
}

ModelDataType::ModelDataType(
        const std::string& bytes)
    : GenericDataType(bytes)
{
    // Do nothing
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
