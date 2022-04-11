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
 * @file GenericAmlipNode.cpp
 */

#include <amlip_node/node/GenericAmlipNode.hpp>
#include <amlip_node/types/Dump.hpp>
#include <node/GenericAmlipNodeImpl.hpp>

namespace eprosima {
namespace amlip {
namespace node {

GenericAmlipNode::GenericAmlipNode()
    : impl_(new GenericAmlipNodeImpl(
        [&]
        (types::GenericType data)
        {
            std::cout << "Data received." << std::endl;
        }
    ))
{
}

GenericAmlipNode::~GenericAmlipNode()
{
    delete impl_;
}

void GenericAmlipNode::publish(types::GenericType& data)
{
    impl_->publish(data);
}


void GenericAmlipNode::publish_vec(std::vector<uint8_t> vec)
{
    types::GenericType data_generic(static_cast<void*>(vec.data()), sizeof(uint8_t)*vec.size());
    impl_->publish(data_generic);
}

bool GenericAmlipNode::wait_writer_matched()
{
    return impl_->wait_writer_matched();
}

std::shared_ptr<types::GenericType> GenericAmlipNode::receive()
{
    return impl_->receive();
}

std::vector<uint8_t> GenericAmlipNode::receive_vec()
{
    std::shared_ptr<types::GenericType> received_data = impl_->receive();
    if (received_data->data() != nullptr)
    {
        const uint8_t* buffer = (uint8_t*)received_data->data();
        std::vector<uint8_t> vec(buffer, buffer + (received_data->data_size()/ sizeof(uint8_t)));
        return vec;
    }
    else
    {
        return std::vector<uint8_t>();
    }
}

types::Dump GenericAmlipNode::receive_dump()
{
    std::shared_ptr<types::GenericType> received_data = impl_->receive();
    if (received_data->data() != nullptr)
    {
        const uint8_t* buffer = (uint8_t*)received_data->data();
        std::vector<uint8_t> vec(buffer, buffer + (received_data->data_size()/ sizeof(uint8_t)));
        return types::Dump(vec);
    }
    else
    {
        return types::Dump();
    }
}

void GenericAmlipNode::spin()
{
    impl_->spin();
}

void GenericAmlipNode::stop()
{
    impl_->stop();
}

types::AmlipId GenericAmlipNode::id() const noexcept
{
    return impl_->id();
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
