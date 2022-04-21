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
 * @file MsReferenceDataType.cpp
 */

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <utility>

#include <types/MsReferenceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* MsReferenceDataType::DATA_TYPE_NAME_ = "ms_reference";

MsReferenceDataType::MsReferenceDataType()
{
}

MsReferenceDataType::MsReferenceDataType(
        const AmlipIdDataType source_id,
        const TaskId& task_id,
        const AmlipIdDataType& target_id)
    : MsRequestDataType(source_id, task_id)
    , target_id_(target_id)
{
}

MsReferenceDataType::~MsReferenceDataType()
{
}

MsReferenceDataType::MsReferenceDataType(
        const MsReferenceDataType& x)
{
    source_id_ = x.source_id_;
    task_id_ = x.task_id_;
    target_id_ = x.target_id_;
}

MsReferenceDataType::MsReferenceDataType(
        MsReferenceDataType&& x)
{
    source_id_ = std::move(x.source_id_);
    task_id_ = std::move(x.task_id_);
    target_id_ = std::move(x.target_id_);
}

MsReferenceDataType& MsReferenceDataType::operator =(
        const MsReferenceDataType& x)
{
    source_id_ = x.source_id_;
    task_id_ = x.task_id_;
    target_id_ = x.target_id_;

    return *this;
}

MsReferenceDataType& MsReferenceDataType::operator =(
        MsReferenceDataType&& x)
{
    source_id_ = std::move(x.source_id_);
    task_id_ = std::move(x.task_id_);
    target_id_ = std::move(x.target_id_);

    return *this;
}

bool MsReferenceDataType::operator ==(
        const MsReferenceDataType& x) const
{
    return (source_id_ == x.source_id_ && task_id_ == x.task_id_ && target_id_ == x.target_id_);
}

bool MsReferenceDataType::operator !=(
        const MsReferenceDataType& x) const
{
    return !(*this == x);
}

bool MsReferenceDataType::operator <(
        const MsReferenceDataType& x) const
{
    if (source_id_ < x.source_id_)
    {
        return true;
    }
    else if (x.source_id_ < source_id_)
    {
        return false;
    }
    else
    {
        if (task_id_ < x.task_id_)
        {
            return true;
        }
        else if (x.task_id_ < task_id_)
        {
            return false;
        }
        else
        {
            return (target_id_ < x.target_id_);
        }
    }
}

AmlipIdDataType MsReferenceDataType::target_id() const
{
    return target_id_;
}

void MsReferenceDataType::target_id(const AmlipIdDataType& new_value)
{
    target_id_ = new_value;
}

void MsReferenceDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << source_id_;
    scdr << task_id_;
    scdr << target_id_;
}

void MsReferenceDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> source_id_;
    dcdr >> task_id_;
    dcdr >> target_id_;
}

void MsReferenceDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t MsReferenceDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);

    return current_alignment - initial_alignment;
}

size_t MsReferenceDataType::get_cdr_serialized_size(
        const MsReferenceDataType& request,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_cdr_serialized_size(request.source_id(), current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += AmlipIdDataType::get_cdr_serialized_size(request.target_id(), current_alignment);

    return current_alignment - initial_alignment;
}

size_t MsReferenceDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool MsReferenceDataType::is_key_defined()
{
    return false;
}

bool MsReferenceDataType::is_bounded()
{
    return true;
}

bool MsReferenceDataType::is_plain()
{
    return true;
}

bool MsReferenceDataType::construct_sample(
        void* memory)
{
    new (memory) MsReferenceDataType();
    return true;
}

const char* MsReferenceDataType::type_name()
{
    return DATA_TYPE_NAME_;
}

std::ostream& operator <<(
        std::ostream& os,
        const MsReferenceDataType& request)
{
    os << "MS-REFERENCE{" << request.source_id() <<
        "|" << request.task_id() <<
        "|" << request.target_id() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
