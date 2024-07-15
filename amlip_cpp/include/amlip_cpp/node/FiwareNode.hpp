// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file FiwareNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_FIWARENODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_FIWARENODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief TODO
 *
 */
class FiwareNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Fiware Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI FiwareNode(
            const char* name,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI FiwareNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI FiwareNode(
            const std::string& name);

    /**
     * @brief Destroy the Fiware Node object and its internal DDS entities.
     *
     */
    AMLIP_CPP_DllAPI ~FiwareNode();
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_FIWARENODE_HPP */
