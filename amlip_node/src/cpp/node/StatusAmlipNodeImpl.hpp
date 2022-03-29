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
 * @file StatusAmlipNodeImpl.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPNODE_STATUSAMLIPNODEIMPL_HPP
#define AMLIP__SRC_CPP_AMLIPNODE_STATUSAMLIPNODEIMPL_HPP

#include <atomic>
#include <functional>
#include <memory>

#include <dds/Reader.hpp>
#include <node/AmlipNodeImpl.hpp>
#include <amlip_node/types/Status.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class StatusAmlipNodeImpl : public AmlipNodeImpl
{
public:

    StatusAmlipNodeImpl(
        std::function<void(types::Status)> callback);

    virtual ~StatusAmlipNodeImpl();

    void spin();

    void stop();

protected:

    types::NodeKind node_kind_() const noexcept override;

    std::shared_ptr<dds::Reader<types::Status>> status_reader_;

    std::function<void(types::Status)> callback_;

    std::atomic<bool> stop_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPNODE_STATUSAMLIPNODEIMPL_HPP */
