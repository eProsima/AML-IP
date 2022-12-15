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
 * @file NodeKind.hpp
 *
 * This is an autogenerated file from command:
 *  enumeration_builder.py
 *   -n eprosima;amlip;types
 *   -e NodeKind
 *   -v undetermined , discovery , agent , main , computing , status , meta
 *   -o .../NodeKind.hpp
 */

#pragma once

#include <array>
#include <string>
#include <stdexcept>
#include <vector>

namespace eprosima {
namespace amlip {
namespace types {

enum class NodeKind
{
    undetermined,
    discovery,
    agent,
    main,
    computing,
    status,
    meta
};

const std::array<std::string, 7> NAMES_NodeKind =
{
    "undetermined",  //! Not defined node kind
    "discovery",  //! Discovery Node [WAN]
    "agent",  //! Agent Node [WAN]
    "main",  //! Main Node [WorkloadDistribution]
    "computing",  //! Computing Node [WorkloadDistribution]
    "status",  //! Status Node
    "meta"  //! MetaNode [ToDo]
};

inline const std::string& to_string(
        const NodeKind& e)
{
    return NAMES_NodeKind[static_cast<int>(e)];
}

inline std::vector<std::string> string_vector_NodeKind()
{
    return std::vector<std::string> (
        NAMES_NodeKind.begin(),
        NAMES_NodeKind.end());
}

inline NodeKind from_string_NodeKind(
        const std::string& s)
{
    for (int i = 0; i < 7; i++)
    {
        if (NAMES_NodeKind[i] == s)
        {
            return static_cast<NodeKind>(i);
        }
    }
    throw std::invalid_argument("Incorrect name for enum NodeKind.");
}

inline std::ostream& operator <<(
        std::ostream& os,
        const NodeKind& e)
{
    os << to_string(e);
    return os;
}

constexpr const unsigned int N_VALUES_NodeKind = 7;

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */