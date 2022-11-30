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

#include <memory>

#include <amlip_cpp/types/status/StatusDataType.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

class Participant;

template <typename T>
class Writer;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief This is an abstract class for a Node
 *
 * This class implement the generic behaviour of every node. This is:
 *
 * - Has a unique Id
 * - Has a state
 * - Has a Node Kind
 *
 * - Holds a dds::Participant with the DdsHandler and every internal entity.
 * - Holds a Status Writer that writes the Status when it changes
 *
 * @attention this is an abstract class, not an interface.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ParentNode
{
public:

    //! Copy constructor not allowed
    AMLIP_CPP_DllAPI ParentNode(
            const ParentNode& x) = delete;

    /**
     * @brief Virtual destructor
     *
     * Clears internal variables and destroys DDS entities.
     */
    AMLIP_CPP_DllAPI virtual ~ParentNode();

    //! Get this node Id
    AMLIP_CPP_DllAPI types::AmlipIdDataType id() const noexcept;

    //! Get this node current state
    AMLIP_CPP_DllAPI types::StateKind current_state() const noexcept;

    //! Get this node kind
    AMLIP_CPP_DllAPI virtual types::NodeKind node_kind() const noexcept;

protected:

    AMLIP_CPP_DllAPI ParentNode(
            const char* name,
            types::NodeKind node_kind,
            types::StateKind initial_state);

    /**
     * @brief Construct a new Parent Node object.
     *
     * @param name name of the node
     * @param node_kind node kind
     *
     * @note node_kind is not taken by a virtual method because it is required ctor.
     *
     * @note protected so this class is abstract and cannot be constructed.
     */
    AMLIP_CPP_DllAPI ParentNode(
            const char* name,
            types::NodeKind node_kind);

    //! Same as previous constructor but with a string argument.
    AMLIP_CPP_DllAPI ParentNode(
            const std::string& name,
            types::NodeKind node_kind);

    /**
     * @brief Change the current status of the node.
     *
     * This provoke to send a Status message.
     *
     * @param new_state new current state.
     */
    AMLIP_CPP_DllAPI void change_status_(
            const types::StateKind& new_state) noexcept;

    //! Publish the current status in Status Writer.
    AMLIP_CPP_DllAPI void publish_status_() const noexcept;

    //! Reference to the Participant and internal entities of DDS.
    std::unique_ptr<dds::Participant> participant_;

    //! DataWriter in topic Status
    std::shared_ptr<dds::Writer<types::StatusDataType>> status_writer_;

    //! Current state
    types::StateKind current_state_;

    /**
     * @brief This Node kind
     *
     * @note this is not taken by a virtual method because it could be required in Parent Node constructor.
     */
    types::NodeKind node_kind_;

};

//! \c ParentNode to stream serializator
AMLIP_CPP_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const ParentNode& node);

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_PARENTNODE_HPP */
