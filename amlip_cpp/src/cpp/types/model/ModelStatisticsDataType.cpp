// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file ModelStatisticsDataType.cpp
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <random>
#include <stdlib.h>
#include <utility>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/types/cast.hpp>
#include <cpp_utils/utils.hpp>

#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

#include <fastdds/rtps/common/CdrSerialization.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* ModelStatisticsDataType::TYPE_NAME_ = "AMLIP-MODEL-STATISTICS";

ModelStatisticsDataType::ModelStatisticsDataType()
    : name_("ModelStatisticsDataTypeName")
    , data_(nullptr)
    , data_size_(0)
    , has_been_allocated_(false)
{
    // Do nothing
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const char* name)
    : name_(name)
    , data_(nullptr)
    , data_size_(0)
    , has_been_allocated_(false)
{
    // Do nothing
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const std::string& name)
    : name_(name)
    , data_(nullptr)
    , data_size_(0)
    , has_been_allocated_(false)
{
    // Do nothing
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const std::string& name,
        void* data,
        const uint32_t size,
        bool copy_data /* = true */)
{
    name_ = name;
    if (copy_data)
    {
        data_ = std::malloc(size * sizeof(uint8_t));
        std::memcpy(data_, data, size);
    }
    else
    {
        data_ = data;
    }
    data_size_ = size;

    has_been_allocated_.store(copy_data);
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const std::string& name,
        const std::vector<ByteType>& bytes,
        bool copy_data /* = true */)
    : ModelStatisticsDataType(
        name,
        utils::cast_to_void_ptr(bytes.data()),
        bytes.size(),
        copy_data)
{
    // Do nothing
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const std::string& name,
        const std::string& bytes,
        bool copy_data /* = true */)
    : ModelStatisticsDataType(
        name,
        utils::cast_to_void_ptr(bytes.c_str()),
        bytes.length(),
        copy_data)
{
    // Do nothing
}

ModelStatisticsDataType::~ModelStatisticsDataType()
{
    // In case the data has been allocated from this class, we free it.
    if (has_been_allocated_)
    {
        logDebug(AMLIPCPP_TYPES_GENERIC, "Releasing data allocated.");
        free(data_);
    }
}

ModelStatisticsDataType::ModelStatisticsDataType(
        const ModelStatisticsDataType& x)
{
    name_ = x.name_;

    if (x.has_been_allocated_)
    {
        logWarning(
            AMLIPCPP_TYPES_GENERIC,
            "Copying a GenericDataType with data that has been allocated from this class. The data will be copied.");
        data_size_ = x.data_size_;
        data_ = malloc(data_size_ * sizeof(uint8_t));
        std::memcpy(data_, x.data_, data_size_);
        has_been_allocated_.store(true);
    }
    else
    {
        data_ = x.data_;
        data_size_ = x.data_size_;
        has_been_allocated_.store(false);
    }

    server_id_ = x.server_id_;
}

ModelStatisticsDataType::ModelStatisticsDataType(
        ModelStatisticsDataType&& x)
{
    name_ = std::move(x.name_);

    this->data_ = x.data_;
    this->data_size_ = x.data_size_;
    this->has_been_allocated_.store(x.has_been_allocated_.load());

    server_id_ = std::move(x.server_id_);

    // Restore x
    x.data_ = nullptr;
    x.data_size_ = 0;
    x.has_been_allocated_.store(false);
}

ModelStatisticsDataType& ModelStatisticsDataType::operator =(
        const ModelStatisticsDataType& x)
{
    name_ = x.name_;

    if (this->has_been_allocated_)
    {
        free(data_);
    }

    if (x.has_been_allocated_)
    {
        logWarning(
            AMLIPCPP_TYPES_GENERIC,
            "Copying a GenericDataType with data that has been allocated from this class. The data will be copied.");
        data_size_ = x.data_size_;
        data_ = malloc(data_size_ * sizeof(uint8_t));
        std::memcpy(data_, x.data_, data_size_);
        has_been_allocated_.store(true);
    }
    else
    {
        data_ = x.data_;
        data_size_ = x.data_size_;
        has_been_allocated_.store(false);
    }

    server_id_ = x.server_id_;

    return *this;
}

ModelStatisticsDataType& ModelStatisticsDataType::operator =(
        ModelStatisticsDataType&& x)
{
    name_ = std::move(x.name_);

    if (this->has_been_allocated_)
    {
        free(data_);
    }

    this->data_ = x.data_;
    this->data_size_ = x.data_size_;
    this->has_been_allocated_.store(x.has_been_allocated_.load());

    server_id_ = std::move(x.server_id_);

    // Restore x
    x.data_ = nullptr;
    x.data_size_ = 0;
    x.has_been_allocated_.store(false);


    return *this;
}

bool ModelStatisticsDataType::operator ==(
        const ModelStatisticsDataType& x) const
{
    return (name_ == x.name_ && server_id_ == x.server_id_ && data_ == x.data_ && data_size_ == x.data_size_);
}

bool ModelStatisticsDataType::operator !=(
        const ModelStatisticsDataType& x) const
{
    return !(*this == x);
}

bool ModelStatisticsDataType::operator <(
        const ModelStatisticsDataType& x) const
{
    if (name_ < x.name_)
    {
        return true;
    }
    else if (name_ > x.name_)
    {
        return false;
    }
    else
    {
        return (server_id_ < x.server_id_);
    }
}

std::string ModelStatisticsDataType::name() const
{
    return name_;
}

std::string& ModelStatisticsDataType::name()
{
    return name_;
}

void ModelStatisticsDataType::name(
        const std::string& name)
{
    name_ = name;
}

void ModelStatisticsDataType::data(
        void* data)
{
    data_ = data;
}

void* ModelStatisticsDataType::data() const
{
    return data_;
}

std::string ModelStatisticsDataType::to_string() const noexcept
{
    return std::string(static_cast<char*>(data_), data_size_);
}

std::vector<ByteType> ModelStatisticsDataType::to_vector() const noexcept
{
    ByteType* buffer = static_cast<ByteType*>(data_);
    return std::vector<ByteType>(buffer, buffer + data_size_);
}

uint32_t ModelStatisticsDataType::data_size() const
{
    return data_size_;
}

uint32_t& ModelStatisticsDataType::data_size()
{
    return data_size_;
}

AmlipIdDataType ModelStatisticsDataType::server_id() const
{
    return server_id_;
}

AmlipIdDataType& ModelStatisticsDataType::server_id()
{
    return server_id_;
}

void ModelStatisticsDataType::server_id(
        const AmlipIdDataType& id)
{
    server_id_ = id;
}

std::string ModelStatisticsDataType::type_name()
{
    return TYPE_NAME_;
}

bool ModelStatisticsDataType::has_been_allocated() const
{
    return has_been_allocated_.load();
}

void ModelStatisticsDataType::has_been_allocated(
        bool take_ownership)
{
    has_been_allocated_.store(take_ownership);
}

void ModelStatisticsDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t ModelStatisticsDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               //
    current_alignment += ((STATISTICS_NAME_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);    // name

    current_alignment += ((DEFAULT_PREALLOCATED_SIZE_) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);      // data
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               // data_size

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               // has_been_allocated_
    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);                           // server_id_

    return current_alignment - initial_alignment;
}

size_t ModelStatisticsDataType::get_cdr_serialized_size(
        const ModelStatisticsDataType& data,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               //
    current_alignment += ((STATISTICS_NAME_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);    // name

    if (data.data_size() > 0)
    {
        current_alignment += ((data.data_size()) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);    // data_
    }
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               // data_size

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                               // has_been_allocated_
    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);                           // server_id_

    return current_alignment - initial_alignment;
}

size_t ModelStatisticsDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool ModelStatisticsDataType::is_key_defined()
{
    return false;
}

bool ModelStatisticsDataType::is_bounded()
{
    return false;
}

bool ModelStatisticsDataType::is_plain()
{
    return false;
}

bool ModelStatisticsDataType::construct_sample(
        void* memory)
{
    return false;
}

std::ostream& operator <<(
        std::ostream& os,
        const ModelStatisticsDataType& id)
{
    // TODO do this from utils using container_to_stream to remove final '.'
    os << "Name {" << id.name() << "}"
       << " Data {" << std::string(static_cast<char*>(id.data()), id.data_size()) << "}"
       << " Server Id {" << id.server_id().to_string() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include auxiliary functions like for serializing/deserializing.
#include  <types/model/impl/ModelStatisticsDataTypeCdrAux.ipp>
