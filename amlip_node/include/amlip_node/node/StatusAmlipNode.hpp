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
 * @file StatusAmlipNode.hpp
 */

#ifndef AMLIP_AMLIPNODE_STATUSAMLIPNODE_HPP
#define AMLIP_AMLIPNODE_STATUSAMLIPNODE_HPP

#include <functional>

#include <amlip_node/types/AmlipId.hpp>
#include <amlip_node/types/Status.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class StatusAmlipNodeImpl;

class StatusAmlipNodeFunctor
{
public:
    virtual bool operator() (types::Status status) const = 0;
};

class StatusAmlipNode
{
public:

    StatusAmlipNode();

    StatusAmlipNode(std::function<void(types::Status)> callback);

    StatusAmlipNode(
        const StatusAmlipNodeFunctor& callback);

    virtual ~StatusAmlipNode();

    void spin();

    void stop();

    types::AmlipId id() const noexcept;

protected:

    StatusAmlipNodeImpl* impl_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP_AMLIPNODE_STATUSAMLIPNODE_HPP */
