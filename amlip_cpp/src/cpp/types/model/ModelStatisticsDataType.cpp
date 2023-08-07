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


#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <utility>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/types/cast.hpp>
#include <cpp_utils/utils.hpp>

#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const size_t ModelStatisticsDataType::DEFAULT_PREALLOCATED_SIZE_ = 16;
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
    : name_(name)
    , data_size_(size)
    , has_been_allocated_(copy_data)
{
    if (copy_data)
    {
        data_ = std::malloc(size * sizeof(uint8_t));
        std::memcpy(data_, data, size);
    }
    else
    {
        data_ = data;
    }
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
    server_id_ = x.server_id_;

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
}

ModelStatisticsDataType::ModelStatisticsDataType(
        ModelStatisticsDataType&& x)
{
    name_ = std::move(x.name_);
    server_id_ = std::move(x.server_id_);

    this->data_ = x.data_;
    this->data_size_ = x.data_size_;
    this->has_been_allocated_.store(x.has_been_allocated_.load());

    // Restore x
    x.data_ = nullptr;
    x.data_size_ = 0;
    x.has_been_allocated_.store(false);
}

ModelStatisticsDataType& ModelStatisticsDataType::operator =(
        const ModelStatisticsDataType& x)
{

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

    name_ = x.name_;
    server_id_ = x.server_id_;

    return *this;
}

ModelStatisticsDataType& ModelStatisticsDataType::operator =(
        ModelStatisticsDataType&& x)
{
    if (this->has_been_allocated_)
    {
        free(data_);
    }

    this->data_ = x.data_;
    this->data_size_ = x.data_size_;
    this->has_been_allocated_.store(x.has_been_allocated_.load());

    // Restore x
    x.data_ = nullptr;
    x.data_size_ = 0;
    x.has_been_allocated_.store(false);

    name_ = std::move(x.name_);
    server_id_ = std::move(x.server_id_);

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

void ModelStatisticsDataType::name(
        const std::string& name)
{
    name_ = name;
}

AmlipIdDataType ModelStatisticsDataType::server_id() const
{
    return server_id_;
}

void ModelStatisticsDataType::server_id(
        const AmlipIdDataType& id)
{
    server_id_ = id;
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

std::string ModelStatisticsDataType::type_name()
{
    return TYPE_NAME_;
}

void ModelStatisticsDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << name_;
    scdr << server_id_;

    scdr << data_size_;
    scdr.serializeArray(static_cast<uint8_t*>(data_), data_size_);
}

void ModelStatisticsDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> name_;
    dcdr >> server_id_;

    // If data has been already allocated (it has been already deserialized), we free it
    if (has_been_allocated_)
    {
        free(data_);
    }

    dcdr >> data_size_;

    // Store enough space to deserialize the data
    data_ = std::malloc(data_size_ * sizeof(uint8_t));
    // Deserialize array
    dcdr.deserializeArray(static_cast<uint8_t*>(data_), data_size_);

    // Set as this data has been allocated by this class
    has_been_allocated_.store(true);
}

void ModelStatisticsDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t ModelStatisticsDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                           // data_size_
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                           // take_ownership
    // It needs an upper bound, but it will not be used
    current_alignment += DEFAULT_PREALLOCATED_SIZE_ + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);  // data

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);                      // server_id_
    current_alignment += (28) + eprosima::fastcdr::Cdr::alignment(current_alignment, 28);                      // name

    return current_alignment - initial_alignment;
}

size_t ModelStatisticsDataType::get_cdr_serialized_size(
        const ModelStatisticsDataType& data,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                           // data_size_
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);                           // take_ownership

    if (data.data_size() > 0)
    {
        current_alignment += data.data_size() + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);                      // server_id_
    current_alignment += (28) + eprosima::fastcdr::Cdr::alignment(current_alignment, 28);                      // name

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
       << " Server Id {" << id.server_id().to_string() << "}"
       << " Data {" << std::string(static_cast<char*>(id.data()), id.data_size()) << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
