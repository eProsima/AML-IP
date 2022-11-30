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

#include <types/multiservice/MsReferenceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* MsReferenceDataType::DATA_TYPE_NAME_ = "ms_reference";

MsReferenceDataType::MsReferenceDataType()
{
}

MsReferenceDataType::MsReferenceDataType(
        const AmlipIdDataType client_id,
        const TaskId& task_id,
        const AmlipIdDataType& server_id)
    : MsRequestDataType(client_id, task_id)
    , server_id_(server_id)
{
}

MsReferenceDataType::~MsReferenceDataType()
{
}

MsReferenceDataType::MsReferenceDataType(
        const MsReferenceDataType& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;
    server_id_ = x.server_id_;
}

MsReferenceDataType::MsReferenceDataType(
        MsReferenceDataType&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);
    server_id_ = std::move(x.server_id_);
}

MsReferenceDataType& MsReferenceDataType::operator =(
        const MsReferenceDataType& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;
    server_id_ = x.server_id_;

    return *this;
}

MsReferenceDataType& MsReferenceDataType::operator =(
        MsReferenceDataType&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);
    server_id_ = std::move(x.server_id_);

    return *this;
}

bool MsReferenceDataType::operator ==(
        const MsReferenceDataType& x) const
{
    return (client_id_ == x.client_id_ && task_id_ == x.task_id_ && server_id_ == x.server_id_);
}

bool MsReferenceDataType::operator !=(
        const MsReferenceDataType& x) const
{
    return !(*this == x);
}

bool MsReferenceDataType::operator <(
        const MsReferenceDataType& x) const
{
    if (client_id_ < x.client_id_)
    {
        return true;
    }
    else if (x.client_id_ < client_id_)
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
            return (server_id_ < x.server_id_);
        }
    }
}

AmlipIdDataType MsReferenceDataType::server_id() const
{
    return server_id_;
}

void MsReferenceDataType::server_id(
        const AmlipIdDataType& new_value)
{
    server_id_ = new_value;
}

void MsReferenceDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    MsRequestDataType::serialize(scdr);
    scdr << server_id_;
}

void MsReferenceDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    MsRequestDataType::deserialize(dcdr);
    dcdr >> server_id_;
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
    // As the data type is plain, the max size and the size for a data is the same
    return get_max_cdr_serialized_size(current_alignment);
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

std::string MsReferenceDataType::type_name()
{
    return DATA_TYPE_NAME_;
}

MsRequestDataType MsReferenceDataType::request() const
{
    return MsRequestDataType(*this);
}

std::ostream& operator <<(
        std::ostream& os,
        const MsReferenceDataType& reference)
{
    os << "MS-REFERENCE{" << reference.client_id() <<
        "|" << reference.task_id() <<
        "|" << reference.server_id() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
