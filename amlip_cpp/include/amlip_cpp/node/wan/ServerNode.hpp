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
 * @file ServerNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_AGENT_SERVERNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_AGENT_SERVERNODE_HPP

#include <ddsrouter_core/types/address/Address.hpp>

#include <amlip_cpp/node/wan/AgentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

/**
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ServerNode : public AgentNode
{
public:

    ServerNode(
            const char* name,
            const std::set<ddsrouter::core::types::Address>& listening_addresses,
            const uint32_t domain_id);

    ServerNode(
            const char* name,
            const std::set<ddsrouter::core::types::Address>& listening_addresses);

protected:

    static ddsrouter::core::configuration::DDSRouterConfiguration get_router_configuration_(
            const char* name,
            const std::set<ddsrouter::core::types::Address>& listening_addresses,
            const uint32_t domain_id);

};

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_AGENT_SERVERNODE_HPP */
