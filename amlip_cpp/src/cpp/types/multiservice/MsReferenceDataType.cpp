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

#include <types/multiservice/MsReferenceDataType.hpp>

#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <utility>

#include <fastdds/rtps/common/CdrSerialization.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* MsReferenceDataType::TYPE_NAME_ = "ms_reference";

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

AmlipIdDataType& MsReferenceDataType::server_id()
{
    return server_id_;
}

void MsReferenceDataType::server_id(
        const AmlipIdDataType& new_value)
{
    server_id_ = new_value;
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
    return TYPE_NAME_;
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

// Include auxiliary functions like for serializing/deserializing.
#include  <types/multiservice/impl/MsReferenceDataTypeCdrAux.ipp>
