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
 * @file AgentNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_AGENT_AGENTNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_AGENT_AGENTNODE_HPP

#include <memory>

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/core/DDSRouter.hpp>

#include <amlip_cpp/node/ParentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

/**
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class AgentNode : public ParentNode
{
public:

    virtual ~AgentNode();

    static ddsrouter::core::configuration::DDSRouterConfiguration default_router_configuration();

protected:

    AgentNode(
            const char* name,
            const ddsrouter::core::configuration::DDSRouterConfiguration& ddsrouter_configuration);

    std::unique_ptr<ddsrouter::core::DDSRouter> router_;
};

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_AGENT_AGENTNODE_HPP */
