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
 * @file GenericDataType.cpp
 * This source file contains the definition of a generic type that contains void* data.
 */

#ifdef _WINID_SIZE
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WINID_SIZE

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <array>
#include <stdlib.h>

#include <types/GenericDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* GenericDataType::TYPE_NAME_ = "GENERIC";
const size_t GenericDataType::DEFAULT_PREALLOCATED_SIZE_ = 10;

GenericDataType::GenericDataType(
        void* data,
        const uint32_t size)
    : data_(data)
    , data_size_(size)
    , has_been_allocated_(false)
{
}

GenericDataType::GenericDataType()
    : GenericDataType(nullptr, 0)
{
}

GenericDataType::~GenericDataType()
{
    // In case the data has been allocated from this class, we free it.
    if (has_been_allocated_)
    {
        free(data_);
    }
}

GenericDataType::GenericDataType(
        const GenericDataType& x)
{
    data_ = x.data_;
    data_size_ = x.data_size_;
}

GenericDataType::GenericDataType(
        GenericDataType&& x)
{
    data_ = std::move(x.data_);
    data_size_ = std::move(x.data_size_);
}

GenericDataType& GenericDataType::operator =(
        const GenericDataType& x)
{
    data_ = x.data_;
    data_size_ = x.data_size_;

    return *this;
}

GenericDataType& GenericDataType::operator =(
        GenericDataType&& x)
{
    data_ = std::move(x.data_);
    data_size_ = x.data_size_;

    return *this;
}

bool GenericDataType::operator ==(
        const GenericDataType& x) const
{
    return (data_ == x.data_ && data_size_ == x.data_size_);
}

bool GenericDataType::operator !=(
        const GenericDataType& x) const
{
    return !(*this == x);
}

void* GenericDataType::data() const
{
    return data_;
}

uint32_t GenericDataType::data_size() const
{
    return data_size_;
}

const char* GenericDataType::type_name()
{
    return TYPE_NAME_;
}

void GenericDataType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << data_size_;
    scdr.serializeArray(static_cast<uint8_t*>(data_), data_size_);
}

void GenericDataType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    // If data has been already allocated (it has been already deserialized), we free it
    if (has_been_allocated_)
    {
        free(data_);
    }

    dcdr >> data_size_;

    // Store enough space to deserialize the data
    data_ = malloc(data_size_ * sizeof(uint8_t));
    // Deserialize array
    dcdr.deserializeArray(static_cast<uint8_t*>(data_), data_size_);

    // Set as this data has been allocated by this class
    has_been_allocated_.store(true);
}

void GenericDataType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t GenericDataType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    // It needs an upper bound, but it will not be used
    current_alignment += DEFAULT_PREALLOCATED_SIZE_ + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);

    return current_alignment - initial_alignment;
}

size_t GenericDataType::get_cdr_serialized_size(
        const GenericDataType& data,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.data_size() > 0)
    {
        current_alignment += data.data_size() + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    return current_alignment - initial_alignment;
}

size_t GenericDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool GenericDataType::is_key_defined()
{
    return false;
}

bool GenericDataType::is_bounded()
{
    return false;
}

bool GenericDataType::is_plain()
{
    return false;
}

bool GenericDataType::construct_sample(
        void* memory)
{
    return false;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
