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
 * @file GenericType.cpp
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

#include <types/GenericType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

GenericType::GenericType(
        void* data,
        const uint32_t size)
    : data_(data)
    , data_size_(size)
    , has_been_allocated_(false)
{
}

GenericType::GenericType()
    : GenericType(nullptr, 0)
{
}

GenericType::~GenericType()
{
    // In case the data has been allocated from this class, we free it.
    if (has_been_allocated_)
    {
        free(data_);
    }
}

GenericType::GenericType(
        const GenericType& x)
{
    data_ = x.data_;
    data_size_ = x.data_size_;
}

GenericType::GenericType(
        GenericType&& x)
{
    data_ = std::move(x.data_);
    data_size_ = std::move(x.data_size_);
}

GenericType& GenericType::operator =(
        const GenericType& x)
{
    data_ = x.data_;
    data_size_ = x.data_size_;

    return *this;
}

GenericType& GenericType::operator =(
        GenericType&& x)
{
    data_ = std::move(x.data_);
    data_size_ = x.data_size_;

    return *this;
}

bool GenericType::operator ==(
        const GenericType& x) const
{
    return (data_ == x.data_ && data_size_ == x.data_size_);
}

bool GenericType::operator !=(
        const GenericType& x) const
{
    return !(*this == x);
}

void* GenericType::data() const
{
    return data_;
}

uint32_t GenericType::data_size() const
{
    return data_size_;
}

const char* GenericType::type_name()
{
    return "GENERIC";
}

void GenericType::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << data_size_;
    scdr.serializeArray(static_cast<uint8_t*>(data_), data_size_);
}

void GenericType::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    // If data has been already allocated (it has been already deserialized), we free it
    if (has_been_allocated_)
    {
        free(data_);
    }

    dcdr >> data_size_;

    // Store enough space to deserialize the data
    data_ = malloc(data_size_);
    // Deserialize array
    dcdr.deserializeArray(static_cast<uint8_t*>(data_), data_size_);

    // Set as this data has been allocated by this class
    has_been_allocated_.store(true);
}

void GenericType::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

size_t GenericType::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    // It needs an upper bound, but it will not be used
    current_alignment += (100 * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);

    return current_alignment - initial_alignment;
}

size_t GenericType::get_cdr_serialized_size(
        const GenericType& data,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    if (data.data_size() > 0)
    {
        current_alignment += (data.data_size() * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    return current_alignment - initial_alignment;
}

size_t GenericType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

bool GenericType::is_key_defined()
{
    return false;
}

bool GenericType::is_bounded()
{
    return false;
}

bool GenericType::is_plain()
{
    return false;
}

bool GenericType::construct_sample(
        void* memory)
{
    return new (memory) GenericType();
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
