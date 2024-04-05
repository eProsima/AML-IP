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

#include <amlip_cpp/types/GenericDataType.hpp>

#include <array>
#include <stdlib.h>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/types/cast.hpp>
#include <cpp_utils/utils.hpp>

#include <fastdds/rtps/common/CdrSerialization.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const char* GenericDataType::TYPE_NAME_ = "GENERIC";
uint32_t GenericDataType::max_cdr_typesize_ = 132UL;

GenericDataType::GenericDataType(
        void* data,
        const uint32_t size,
        bool take_ownership /* = false */)
{
    data_size_ = size;
    data_ = malloc(size * sizeof(uint8_t));
    std::memcpy(data_, data, size);
    has_been_allocated_.store(take_ownership);
}

GenericDataType::GenericDataType()
    : GenericDataType(nullptr, 0)
{
}

GenericDataType::GenericDataType(
        const std::vector<ByteType>& bytes)
    : GenericDataType(
        utils::copy_to_void_ptr(utils::cast_to_void_ptr(bytes.data()), bytes.size()),
        bytes.size(),
        true)
{
    // Do nothing
}

GenericDataType::GenericDataType(
        const std::string& bytes)
    : GenericDataType(
        utils::copy_to_void_ptr(utils::cast_to_void_ptr(bytes.c_str()), bytes.length()),
        bytes.length(),
        true)
{
    // Do nothing
}

GenericDataType::~GenericDataType()
{
    // In case the data has been allocated from this class, we free it.
    if (has_been_allocated_)
    {
        logDebug(AMLIPCPP_TYPES_GENERIC, "Releasing data allocated.");
        free(data_);
    }
}

GenericDataType::GenericDataType(
        const GenericDataType& x)
{
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

GenericDataType::GenericDataType(
        GenericDataType&& x)
{
    this->data_ = x.data_;
    this->data_size_ = x.data_size_;
    this->has_been_allocated_.store(x.has_been_allocated_.load());

    // Restore x
    x.data_ = nullptr;
    x.data_size_ = 0;
    x.has_been_allocated_.store(false);
}

GenericDataType& GenericDataType::operator =(
        const GenericDataType& x)
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

    return *this;
}

GenericDataType& GenericDataType::operator =(
        GenericDataType&& x)
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

void GenericDataType::data(
        void* data)
{
    data_ = data;
}

void* GenericDataType::data() const
{
    return data_;
}

uint32_t GenericDataType::data_size() const
{
    return data_size_;
}

uint32_t& GenericDataType::data_size()
{
    return data_size_;
}

std::string GenericDataType::type_name()
{
    return TYPE_NAME_;
}

bool GenericDataType::has_been_allocated() const
{
    return has_been_allocated_.load();
}

void GenericDataType::has_been_allocated(
        bool take_ownership)
{
    has_been_allocated_.store(take_ownership);
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

std::string GenericDataType::to_string() const noexcept
{
    return std::string(static_cast<char*>(data_), data_size_);
}

std::vector<ByteType> GenericDataType::to_vector() const noexcept
{
    ByteType* buffer = static_cast<ByteType*>(data_);
    return std::vector<ByteType>(buffer, buffer + data_size_);
}

std::ostream& operator <<(
        std::ostream& os,
        const GenericDataType& obj)
{
    os << "DATA{" << obj.to_string() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include auxiliary functions like for serializing/deserializing.
#include <types/impl/GenericDataTypeCdrAux.ipp>
