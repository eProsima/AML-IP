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
 * @file Dump.cpp
 */

#include <iterator>
#include <vector>

#include <amlip_node/types/Dump.hpp>

namespace eprosima {
namespace amlip {
namespace types {

Dump::Dump()
{
    vec_ = std::vector<uint8_t>();
}

Dump::Dump(std::vector<uint8_t> vec)
{
    vec_ = vec;
}

const char* Dump::get_bytes()
{
    return reinterpret_cast<char*>(vec_.data());
}

size_t Dump::get_size()
{
    return vec_.size();
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
