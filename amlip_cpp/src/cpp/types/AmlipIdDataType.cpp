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
 * @file AmlipIdDataType.cpp
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
#include <utility>
#include <random>
#include <string>

#include <types/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const AmlipIdDataType AmlipIdDataType::UNDEFINED_ID_= AmlipIdDataType();

AmlipIdDataType::AmlipIdDataType()
{
    memset(&name_, 0, (NAME_SIZE) * 1);
    memset(&rand_id_, 0, (RAND_SIZE) * 1);
}

AmlipIdDataType::AmlipIdDataType(const std::string& name)
{
    name_ = str_name_to_array(name);
    memset(&rand_id_, 0, (RAND_SIZE) * 1);
}

AmlipIdDataType::AmlipIdDataType(const char* name)
{
    name_ = str_name_to_array(std::string(name));
    memset(&rand_id_, 0, (RAND_SIZE) * 1);
}

AmlipIdDataType::AmlipIdDataType(
    std::array<uint8_t, NAME_SIZE>& name,
    std::array<uint8_t, RAND_SIZE>& rand_id)
{
    name_ = name;
    rand_id_ = rand_id;
}

AmlipIdDataType::AmlipIdDataType(
    std::array<uint8_t, NAME_SIZE>&& name,
    std::array<uint8_t, RAND_SIZE>&& rand_id)
{
    name_ = std::move(name);
    rand_id_ = std::move(rand_id);
}

AmlipIdDataType::~AmlipIdDataType()
{
}

AmlipIdDataType::AmlipIdDataType(
    const AmlipIdDataType& x)
{
    name_ = x.name_;
    rand_id_ = x.rand_id_;
}

AmlipIdDataType::AmlipIdDataType(
    AmlipIdDataType&& x)
{
    name_ = std::move(x.name_);
    rand_id_ = std::move(x.rand_id_);
}

AmlipIdDataType& AmlipIdDataType::operator =(
    const AmlipIdDataType& x)
{
    name_ = x.name_;
    rand_id_ = x.rand_id_;

    return *this;
}

AmlipIdDataType& AmlipIdDataType::operator =(
    AmlipIdDataType&& x)
{
    name_ = std::move(x.name_);
    rand_id_ = std::move(x.rand_id_);

    return *this;
}

bool AmlipIdDataType::operator ==(
    const AmlipIdDataType& x) const
{
    return (name_ == x.name_ && rand_id_ == x.rand_id_);
}

bool AmlipIdDataType::operator !=(
    const AmlipIdDataType& x) const
{
    return !(*this == x);
}

std::string AmlipIdDataType::name() const
{
    return std::string(std::begin(name_), std::end(name_));
}

void AmlipIdDataType::name(const std::array<uint8_t, NAME_SIZE>& name)
{
    name_ = name;
}

const std::array<uint8_t, NAME_SIZE>& AmlipIdDataType::base64_name() const
{
    return name_;
}

const std::array<uint8_t, RAND_SIZE>& AmlipIdDataType::id() const
{
    return rand_id_;
}

void AmlipIdDataType::id(const std::array<uint8_t, RAND_SIZE>& id)
{
    rand_id_ = id;
}

const char* AmlipIdDataType::type_name()
{
    return "AMLIP-ID";
}

bool AmlipIdDataType::is_defined() const noexcept
{
    return (*this) != UNDEFINED_ID_;
}

AmlipIdDataType AmlipIdDataType::new_unique_id()
{
    // initialize the random number generator
    srand (time(NULL));

    std::array<uint8_t, NAME_SIZE> name;
    for (int i=0; i<NAME_SIZE; i++)
    {
        // generate a random char between 'a' and 'z'
        char c = 'a' + static_cast<char>(rand() % 26);
        name[i] = static_cast<uint8_t>(c);
    }

    std::array<uint8_t, RAND_SIZE> rand_id;
    for (int i=0; i<RAND_SIZE; i++)
    {
        // generate a random number between 0 and 255
        uint8_t num = static_cast<uint8_t>(rand() % 256);
        rand_id[i] = num;
    }

    return AmlipIdDataType(name, rand_id);
}

AmlipIdDataType AmlipIdDataType::new_unique_id(const std::string& name)
{
    AmlipIdDataType new_id = new_unique_id();
    new_id.name(str_name_to_array(name));

    return new_id;
}

AmlipIdDataType AmlipIdDataType::new_unique_id(const char* name)
{
    return new_unique_id(std::string(name));
}

AmlipIdDataType AmlipIdDataType::undefined_id()
{
    return UNDEFINED_ID_;
}

std::array<uint8_t, NAME_SIZE> AmlipIdDataType::str_name_to_array(const std::string& name)
{
    std::array<uint8_t, NAME_SIZE> _name;
    std::string name_substr = name.substr(0, NAME_SIZE);
    std::copy(name_substr.begin(), name_substr.end(), _name.data());

    return _name;
}

void AmlipIdDataType::serialize(
    eprosima::fastcdr::Cdr& scdr) const
{
    scdr << name_;
    scdr << rand_id_;
}

void AmlipIdDataType::deserialize(
    eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> name_;
    dcdr >> rand_id_;
}

void AmlipIdDataType::serialize_key(
    eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
}

size_t AmlipIdDataType::get_max_cdr_serialize_size(
    size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
    current_alignment += ((NAME_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    current_alignment += ((RAND_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);

    return current_alignment - initial_alignment;
}

size_t AmlipIdDataType::get_cdr_serialize_size(
    const AmlipIdDataType& data,
    size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;

    if ((NAME_SIZE) > 0)
    {
        current_alignment += ((NAME_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    if ((RAND_SIZE) > 0)
    {
        current_alignment += ((RAND_SIZE) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    return current_alignment - initial_alignment;
}

size_t AmlipIdDataType::get_key_max_cdr_serialized_size(
    size_t current_alignment)
{
    return current_alignment;
}

bool AmlipIdDataType::is_key_defined()
{
    return false;
}

bool AmlipIdDataType::is_bounded()
{
    return true;
}

bool AmlipIdDataType::is_plain()
{
    return true;
}

bool AmlipIdDataType::construct_sample(
    void* memory)
{
    new (memory) AmlipIdDataType();
    return true;
}

std::ostream& operator <<(
    std::ostream& os,
    const AmlipIdDataType& id)
{
    os << "ID{" << id.name() << "-";
    for (char v : id.id())
    {
        os << v;
    }
    os << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
