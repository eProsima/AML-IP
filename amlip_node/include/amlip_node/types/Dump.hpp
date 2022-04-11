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
 * @file Dump.hpp
 * This header file contains the declaration of a generic type that contains void* data.
 */

#ifndef AMLIP__SRC_CPP_AMLIPTYPES_DUMP_HPP
#define AMLIP__SRC_CPP_AMLIPTYPES_DUMP_HPP

#include <vector>

namespace eprosima {
namespace amlip {
namespace types {

class Dump
{
public:

    Dump();

    Dump(std::vector<uint8_t> vec);

    const char* get_bytes();

    size_t get_size();

protected:

    std::vector<uint8_t> vec_;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_AMLIPTYPES_DUMP_HPP