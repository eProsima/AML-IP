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
 * @file ClientNode.cpp
 */

#include <ddsrouter_core/configuration/DDSRouterConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/SimpleParticipantConfiguration.hpp>
#include <ddsrouter_core/configuration/participant/InitialPeersParticipantConfiguration.hpp>
#include <ddsrouter_core/types/participant/ParticipantKind.hpp>

#include <amlip_cpp/node/agent/ClientNode.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

ClientNode::ClientNode(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& connection_addresses,
        const uint32_t domain_id)
    : AgentNode(name, get_router_configuration_(name, connection_addresses, domain_id))
{
    logInfo(AMLIPCPP_NODE_CLIENT, "Created new Agent Client Node: " << *this << ".");
}

ClientNode::ClientNode(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& connection_addresses)
    : ClientNode(name, connection_addresses, dds::Participant::default_domain_id())
{
}

ddsrouter::core::configuration::DDSRouterConfiguration ClientNode::get_router_configuration_(
        const char* name,
        const std::set<ddsrouter::core::types::Address>& connection_addresses,
        const uint32_t domain_id)
{
    ddsrouter::core::configuration::DDSRouterConfiguration configuration = AgentNode::default_router_configuration();

    // Create Simple internal Participant
    configuration.participants_configurations.insert(
        std::make_shared<ddsrouter::core::configuration::SimpleParticipantConfiguration>(
            std::string("local_") + name,
            ddsrouter::core::types::ParticipantKind::simple_rtps,
            false,
            domain_id));

    // Create WAN Participant as client
    configuration.participants_configurations.insert(
        std::make_shared<ddsrouter::core::configuration::InitialPeersParticipantConfiguration>(
            std::string("wan_client_") + name,
            ddsrouter::core::types::ParticipantKind::wan_initial_peers,
            false,
            dds::Participant::default_domain_id(),
            std::set<ddsrouter::core::types::Address>(),
            connection_addresses,
            ddsrouter::core::types::security::TlsConfiguration()
        ));

    return configuration;
}

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */