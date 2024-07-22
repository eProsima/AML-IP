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
 * @file FiwareNode.cpp
 */

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/node/FiwareNode.hpp>

#include <dds/Participant.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace node {

FiwareNode::FiwareNode(
        const char* name,
        uint32_t domain_id)
    : ParentNode(name, types::NodeKind::fiware, types::StateKind::running, domain_id)
{
    logInfo(AMLIPCPP_NODE_FIWARE, "Created new Fiware Node: " << *this << ".");
}

FiwareNode::FiwareNode(
        const char* name)
    : FiwareNode(name, dds::Participant::default_domain_id())
{
}

FiwareNode::FiwareNode(
        const std::string& name)
    : FiwareNode(name.c_str(), dds::Participant::default_domain_id())
{
}

FiwareNode::~FiwareNode()
{
    change_status_(types::StateKind::stopped);
    logDebug(AMLIPCPP_NODE_FIWARE, "Destroying Fiware Node: " << *this << ".");
}

std::ostream& operator <<(
        std::ostream& os,
        const FiwareNode& node)
{
    os << "FIWARE_NODE{" << node.id() << "}";
    return os;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
