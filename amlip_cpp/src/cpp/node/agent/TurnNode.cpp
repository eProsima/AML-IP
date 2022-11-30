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
 * @file TurnNode.cpp
 */

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/InitialPeersParticipantConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

#include <amlip_cpp/node/agent/TurnNode.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

TurnNode::TurnNode(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& listening_addresses,
        const std::set<ddsrouter::core::types::Address>& connection_addresses)
    : AgentNode(name, get_router_configuration_(name, listening_addresses, connection_addresses))
{
    logInfo(AMLIPCPP_NODE_TURN, "Created new Agent Turn Node: " << *this << ".");
}

TurnNode::TurnNode(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& listening_addresses)
    : TurnNode(name, listening_addresses, {})
{
    // Do nothing
}

ddsrouter::core::configuration::DDSRouterConfiguration TurnNode::get_router_configuration_(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& listening_addresses,
        const std::set<ddsrouter::core::types::Address>& connection_addresses)
{
    ddsrouter::core::configuration::DDSRouterConfiguration configuration = AgentNode::default_router_configuration();

    // Create WAN Participant as Repeater
    configuration.participants_configurations.insert(
        std::make_shared<ddsrouter::core::configuration::InitialPeersParticipantConfiguration>(
            std::string("wan_turn_") + name,
            ddsrouter::core::types::ParticipantKind::wan_initial_peers,
            true,
            dds::Participant::default_domain_id(),
            listening_addresses,
            connection_addresses,
            ddsrouter::core::types::security::TlsConfiguration()
        ));

    return configuration;
}

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
