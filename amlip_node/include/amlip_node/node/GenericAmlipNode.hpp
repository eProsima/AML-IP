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
 * @file GenericAmlipNode.hpp
 */

#ifndef AMLIP_AMLIPNODE_GENERICAMLIPNODE_HPP
#define AMLIP_AMLIPNODE_GENERICAMLIPNODE_HPP

#include <memory>
#include <vector>

#include <amlip_node/types/AmlipId.hpp>
#include <amlip_node/types/Dump.hpp>
#include <amlip_node/types/GenericType.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class GenericAmlipNodeImpl;

class GenericAmlipNode
{
public:

    GenericAmlipNode();

    virtual ~GenericAmlipNode();

    void publish(types::GenericType& data);

    void publish_vec(std::vector<uint8_t> vec);

    bool wait_writer_matched();

    std::shared_ptr<types::GenericType> receive();

    std::vector<uint8_t> receive_vec();

    types::Dump receive_dump();

    void spin();

    void stop();

    types::AmlipId id() const noexcept;

protected:

    GenericAmlipNodeImpl* impl_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP_AMLIPNODE_GENERICAMLIPNODE_HPP */
