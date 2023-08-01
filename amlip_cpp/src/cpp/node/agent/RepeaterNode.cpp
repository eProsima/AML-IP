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
 * @file RepeaterNode.cpp
 */

#include <ddspipe_participants/configuration/InitialPeersParticipantConfiguration.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/types/ParticipantKind.hpp>

#include <amlip_cpp/node/wan/RepeaterNode.hpp>

#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

RepeaterNode::RepeaterNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const std::set<ddspipe::participants::types::Address>& connection_addresses,
        const uint32_t domain_id)
    : AgentNode(name, get_router_configuration_(name, listening_addresses, connection_addresses, domain_id))
{
    logInfo(AMLIPCPP_NODE_REPEATER, "Created new Agent Repeater Node: " << *this << ".");
}

RepeaterNode::RepeaterNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const std::set<ddspipe::participants::types::Address>& connection_addresses)
    : RepeaterNode(name, listening_addresses, connection_addresses, dds::Participant::default_domain_id())
{
    // Do nothing
}

RepeaterNode::RepeaterNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const uint32_t domain_id)
    : RepeaterNode(name, listening_addresses, {}, domain_id)
{
    // Do nothing
}

RepeaterNode::RepeaterNode(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses)
    : RepeaterNode(name, listening_addresses, {}, dds::Participant::default_domain_id())
{
    // Do nothing
}

ddsrouter::core::DdsRouterConfiguration RepeaterNode::get_router_configuration_(
        const char* name,
        const std::set<ddspipe::participants::types::Address>& listening_addresses,
        const std::set<ddspipe::participants::types::Address>& connection_addresses,
        const uint32_t domain_id)
{
    ddsrouter::core::DdsRouterConfiguration configuration = AgentNode::default_router_configuration();

    // Create WAN Participant as Repeater
    {
        auto conf = std::make_shared<ddspipe::participants::InitialPeersParticipantConfiguration>();
        conf->id = std::string("wan_repeater_") + name;
        conf->is_repeater = true;
        conf->domain = domain_id;
        conf->listening_addresses = listening_addresses;
        conf->connection_addresses = connection_addresses;

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
