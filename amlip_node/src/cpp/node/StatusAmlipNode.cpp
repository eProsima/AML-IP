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
 * @file StatusAmlipNode.cpp
 */

#include <node/StatusAmlipNodeImpl.hpp>
#include <amlip_node/node/StatusAmlipNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

StatusAmlipNode::StatusAmlipNode()
    : StatusAmlipNode(
        [&]
        (types::Status status)
        {
            std::cout << "Status read: " << status << std::endl;
        }
    )
{
}

StatusAmlipNode::StatusAmlipNode(std::function<void(types::Status)> callback)
    : impl_(new StatusAmlipNodeImpl(callback))
{
}

StatusAmlipNode::StatusAmlipNode(const StatusAmlipNodeFunctor& callback)
    : impl_(new StatusAmlipNodeImpl(
        [&callback]
        (types::Status status)
        {
            std::cout << "CALLED callback in StatusAmlipNodeFunctor" << std::endl;
            callback(status);
        }))
{
    std::cout << "Created with StatusAmlipNodeFunctor" << std::endl;
}

StatusAmlipNode::~StatusAmlipNode()
{
    delete impl_;
}

void StatusAmlipNode::spin()
{
    impl_->spin();
}

void StatusAmlipNode::stop()
{
    impl_->stop();
}

types::AmlipId StatusAmlipNode::id() const noexcept
{
    return impl_->id();
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
