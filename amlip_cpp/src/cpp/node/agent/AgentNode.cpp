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
 * @file AgentNode.cpp
 */

#include <cpp_utils/Log.hpp>

#include <ddspipe_core/types/topic/filter/WildcardDdsFilterTopic.hpp>

#include <ddsrouter_core/configuration/DdsRouterConfiguration.hpp>
#include <ddsrouter_core/core/DdsRouter.hpp>

#include <amlip_cpp/node/wan/AgentNode.hpp>

#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace agent {

AgentNode::AgentNode(
        const char* name,
        const ddsrouter::core::DdsRouterConfiguration& ddsrouter_configuration)
    : ParentNode(name, types::NodeKind::agent)
    , router_(std::make_unique<ddsrouter::core::DdsRouter>(ddsrouter_configuration))
{
    logInfo(AMLIPCPP_NODE_AGENT, "Created new Agent Node: " << *this << ".");

    router_->start();
    change_status_(types::StateKind::running);
}

AgentNode::~AgentNode()
{
    logDebug(AMLIPCPP_NODE_AGENT, "Destroying Agent Node: " << *this << ".");
}

ddsrouter::core::DdsRouterConfiguration AgentNode::default_router_configuration()
{
    ddsrouter::core::DdsRouterConfiguration configuration;

    // Set allowlist
    ddspipe::core::types::WildcardDdsFilterTopic amlip_topic;
    amlip_topic.topic_name.set_value(std::string(dds::utils::TOPIC_NAME_MANGLING) + "*");
    configuration.allowlist.insert(
            utils::Heritable<ddspipe::core::types::WildcardDdsFilterTopic>::make_heritable(amlip_topic));

    return configuration;
}

} /* namespace agent */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
