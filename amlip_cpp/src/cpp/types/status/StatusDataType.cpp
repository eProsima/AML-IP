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

/*!
 * @file StatusDataType.cpp
 */

#include <types/status/StatusDataType.hpp>

#include <fastdds/rtps/common/CdrSerialization.hpp>

#include <cpp_utils/utils.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* StatusDataType::TYPE_NAME_ = "status";

StatusDataType::StatusDataType(
        const AmlipIdDataType& id,
        const NodeKind& node_kind,
        const StateKind& state)
    : id_(id)
    , node_kind_(node_kind)
    , state_(state)
{
}

StatusDataType::StatusDataType()
    : StatusDataType(
        AmlipIdDataType(),
        NodeKind::undetermined,
        StateKind::unknown)
{
}

bool StatusDataType::operator ==(
        const StatusDataType& x) const
{
    return (id() == x.id() && node_kind() == x.node_kind() && state() == x.state());
}

bool StatusDataType::operator !=(
        const StatusDataType& x) const
{
    return !(*this == x);
}

AmlipIdDataType StatusDataType::id() const noexcept
{
    return id_;
}

AmlipIdDataType& StatusDataType::id() noexcept
{
    return id_;
}

NodeKind StatusDataType::node_kind() const noexcept
{
    return node_kind_;
}

NodeKind& StatusDataType::node_kind() noexcept
{
    return node_kind_;
}

StateKind StatusDataType::state() const noexcept
{
    return state_;
}

StateKind& StatusDataType::state() noexcept
{
    return state_;
}

bool StatusDataType::is_defined() const noexcept
{
    return (id().is_defined() && node_kind() != NodeKind::undetermined && state() != StateKind::unknown);
}

std::string StatusDataType::to_string() const noexcept
{
    return utils::generic_to_string(*this);
}

bool StatusDataType::is_bounded()
{
    return true;
}

bool StatusDataType::is_plain()
{
    return true;
}

bool StatusDataType::construct_sample(
        void* memory)
{
    return new (memory) StatusDataType();
}

std::string StatusDataType::type_name()
{
    return TYPE_NAME_;
}

std::ostream& operator <<(
        std::ostream& os,
        const StatusDataType& st)
{
    os << "STATUS{" << st.id() << ", " << st.node_kind() << ", " << st.state() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include auxiliary functions like for serializing/deserializing.
#include  <types/status/impl/StatusDataTypeCdrAux.ipp>
