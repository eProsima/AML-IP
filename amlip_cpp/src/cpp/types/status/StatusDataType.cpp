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

#include <fastcdr/Cdr.h>
#include <ddsrouter_utils/utils.hpp>

#include <types/status/StatusDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* StatusDataType::DATA_TYPE_NAME_ = "status";

StatusDataType::StatusDataType(
        AmlipIdDataType id,
        NodeKind node_kind,
        StateKind state)
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

NodeKind StatusDataType::node_kind() const noexcept
{
    return node_kind_;
}

StateKind StatusDataType::state() const noexcept
{
    return state_;
}

bool StatusDataType::is_defined() const noexcept
{
    return (id().is_defined() && node_kind() != NodeKind::undetermined && state() != StateKind::unknown);
}

std::string StatusDataType::to_string() const noexcept
{
    return ddsrouter::utils::to_string(*this);
}

void StatusDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << id_;
    scdr << (uint32_t)node_kind_;
    scdr << (uint32_t)state_;
}

void StatusDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> id_;

    {
        uint32_t enum_value = 0;
        dcdr >> enum_value;
        node_kind_ = (NodeKind)enum_value;
    }

    {
        uint32_t enum_value = 0;
        dcdr >> enum_value;
        state_ = (StateKind)enum_value;
    }
}

void StatusDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t StatusDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    return current_alignment - initial_alignment;
}

size_t StatusDataType::get_cdr_serialized_size(
        const StatusDataType&,
        size_t current_alignment)
{
    return get_max_cdr_serialized_size(current_alignment);
}

size_t StatusDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool StatusDataType::is_key_defined()
{
    return false;
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
    return DATA_TYPE_NAME_;
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
