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

#include <types/multiservice/MsRequestDataType.hpp>

#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <utility>

#include <fastdds/rtps/common/CdrSerialization.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* MsRequestDataType::TYPE_NAME_ = "ms_request";

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

AmlipIdDataType& MsRequestDataType::client_id()
{
    return client_id_;
}

void MsRequestDataType::client_id(
        AmlipIdDataType& new_value)
{
    client_id_ = new_value;
}

TaskId MsRequestDataType::task_id() const
{
    return task_id_;
}

TaskId& MsRequestDataType::task_id()
{
    return task_id_;
}

void MsRequestDataType::task_id(
        TaskId& new_value)
{
    task_id_ = new_value;
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
    return TYPE_NAME_;
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

// Include auxiliary functions like for serializing/deserializing.
#include  <types/multiservice/impl/MsRequestDataTypeCdrAux.ipp>
