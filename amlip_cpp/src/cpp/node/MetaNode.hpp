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
 * @file MetaNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_METANODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_METANODE_HPP

#include <ddsrouter_utils/memory/OwnerPtr.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * @brief TODO
 *
 */
class MetaNode
{
public:

};

//! \c MetaNode to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const MetaNode& participant);

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/MetaNode.ipp>

#endif /* AMLIPCPP__SRC_CPP_NODE_METANODE_HPP */
