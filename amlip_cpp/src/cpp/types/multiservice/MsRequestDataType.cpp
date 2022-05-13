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
 * @file MsRequestDataType.cpp
 */

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <utility>

#include <amlip_cpp/types/multiservice/MsRequestDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* MsRequestDataType::DATA_TYPE_NAME_ = "ms_request";

MsRequestDataType::MsRequestDataType()
{
}

MsRequestDataType::MsRequestDataType(
        const AmlipIdDataType client_id,
        const TaskId& task_id)
    : client_id_(client_id)
    , task_id_(task_id)
{
}

MsRequestDataType::~MsRequestDataType()
{
}

MsRequestDataType::MsRequestDataType(
        const MsRequestDataType& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;
}

MsRequestDataType::MsRequestDataType(
        MsRequestDataType&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);
}

MsRequestDataType& MsRequestDataType::operator =(
        const MsRequestDataType& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;

    return *this;
}

MsRequestDataType& MsRequestDataType::operator =(
        MsRequestDataType&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);

    return *this;
}

bool MsRequestDataType::operator ==(
        const MsRequestDataType& x) const
{
    return (client_id_ == x.client_id_ && task_id_ == x.task_id_);
}

bool MsRequestDataType::operator !=(
        const MsRequestDataType& x) const
{
    return !(*this == x);
}

bool MsRequestDataType::operator <(
        const MsRequestDataType& x) const
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
        return (task_id_ < x.task_id_);
    }
}

AmlipIdDataType MsRequestDataType::client_id() const
{
    return client_id_;
}

void MsRequestDataType::client_id(
        const AmlipIdDataType& new_value)
{
    client_id_ = new_value;
}

TaskId MsRequestDataType::task_id() const
{
    return task_id_;
}

void MsRequestDataType::task_id(
        const TaskId& new_value)
{
    task_id_ = new_value;
}

void MsRequestDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << client_id_;
    scdr << task_id_;
}

void MsRequestDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> client_id_;
    dcdr >> task_id_;
}

void MsRequestDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t MsRequestDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    return current_alignment - initial_alignment;
}

size_t MsRequestDataType::get_cdr_serialized_size(
        const MsRequestDataType&,
        size_t current_alignment)
{
    // As the data type is plain, the max size and the size for a data is the same
    return get_max_cdr_serialized_size(current_alignment);
}

size_t MsRequestDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool MsRequestDataType::is_key_defined()
{
    return false;
}

bool MsRequestDataType::is_bounded()
{
    return true;
}

bool MsRequestDataType::is_plain()
{
    return true;
}

bool MsRequestDataType::construct_sample(
        void* memory)
{
    new (memory) MsRequestDataType();
    return true;
}

std::string MsRequestDataType::type_name()
{
    return DATA_TYPE_NAME_;
}

std::ostream& operator <<(
        std::ostream& os,
        const MsRequestDataType& request)
{
    os << "MS-REQUEST{" << request.client_id() << "|" << request.task_id() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
