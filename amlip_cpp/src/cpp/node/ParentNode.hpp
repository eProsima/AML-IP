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
 * @file ParentNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_PARENTNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_PARENTNODE_HPP

#include <cpp_utils/memory/owner_ptr.hpp>
#include <cpp_utils/ReturnCode.hpp>

#include <types/status/StatusDataType.hpp>
#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ParentNode
{
public:

    //! Copy constructor not allowed
    ParentNode(
            const ParentNode& x) = delete;

    virtual ~ParentNode();

    types::AmlipIdDataType id() const noexcept;

    types::StateKind current_state() const noexcept;

    virtual types::NodeKind node_kind() const noexcept;

protected:

    ParentNode(
            const char* name,
            types::NodeKind node_kind);
    ParentNode(
            const std::string& name,
            types::NodeKind node_kind);

    void change_status_(
            types::StateKind new_state) noexcept;

    void publish_status_() const noexcept;

    dds::Participant participant_;

    std::shared_ptr<dds::Writer<types::StatusDataType>> status_writer_;

    types::StateKind current_state_;

    types::NodeKind node_kind_;

};

//! \c ParentNode to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParentNode& node);

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_PARENTNODE_HPP */