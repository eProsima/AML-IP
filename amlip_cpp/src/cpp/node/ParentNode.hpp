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

#include <ddsrouter_utils/memory/OwnerPtr.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

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

    ParentNode(const char* name);
    ParentNode(std::string name);

    ~ParentNode();

    types::AmlipIdDataType id() const noexcept;

    types::StateKind current_state() const noexcept;

    virtual types::NodeKind node_kind() const noexcept;

    /**
     * @brief TODO
     *
     * @note decorator
     */
    ddsrouter::utils::ReturnCode run();

    /**
     * @brief TODO
     *
     * @note decorator
     */
    ddsrouter::utils::ReturnCode stop();

protected:

    virtual ddsrouter::utils::ReturnCode run_();

    virtual ddsrouter::utils::ReturnCode stop_();

    void publish_status_() const;

    dds::Participant participant_;

    std::shared_ptr<dds::Writer<types::StatusDataType>> status_writer_;

    types::StateKind current_state_;

};

//! \c ParentNode to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ParentNode& node);

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_PARENTNODE_HPP */
