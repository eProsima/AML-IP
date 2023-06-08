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
 * @file ServerNode.cpp
 */

#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>
#include <ddspipe_participants/configuration/SimpleParticipantConfiguration.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

#include <amlip_cpp/node/wan/ServerNode.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

ServerNode::ServerNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const uint32_t domain_id)
    : AgentNode(name, get_router_configuration_(name, listening_addresses, domain_id))
{
    logInfo(AMLIPCPP_NODE_SERVER, "Created new Agent Server Node: " << *this << ".");
}

ServerNode::ServerNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses)
    : ServerNode(name, listening_addresses, dds::Participant::default_domain_id())
{
}

ddsrouter::core::DdsRouterConfiguration ServerNode::get_router_configuration_(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const uint32_t domain_id)
{
    ddsrouter::core::DdsRouterConfiguration configuration = AgentNode::default_router_configuration();

    // Create Simple internal Participant
    {
        auto conf = std::make_shared<ddspipe::participants::SimpleParticipantConfiguration>();
        conf->id = std::string("local_") + name;
        conf->is_repeater = false;
        conf->domain = domain_id;

        configuration.participants_configurations.insert(
            {
                ddsrouter::core::types::ParticipantKind::simple,
                conf
            }
        );
    }

    // Create WAN Participant as Server
    {
        auto conf = std::make_shared<ddspipe::participants::InitialPeersParticipantConfiguration>();
        conf->id = std::string("wan_server_") + name;
        conf->is_repeater = false;
        conf->domain = dds::Participant::default_domain_id();
        conf->listening_addresses = listening_addresses;

        configuration.participants_configurations.insert(
            {
                ddsrouter::core::types::ParticipantKind::initial_peers,
                conf
            }
        );
    }

    return configuration;
}

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
