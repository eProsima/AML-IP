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
char dummy;     // TODO: Check whether this is actually useful and remove if not
}  // namespace
#endif  // _WINID_SIZE

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#include <cpp_utils/utils.hpp>
#include <cpp_utils/math/random/TSafeRandomManager.hpp>

#include <fastdds/rtps/common/CdrSerialization.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const AmlipIdDataType AmlipIdDataType::UNDEFINED_ID_ = AmlipIdDataType();
const char* AmlipIdDataType::TYPE_NAME_ = "AMLIP-ID";

AmlipIdDataType::AmlipIdDataType()
    : name_({0})
    , rand_id_({0})
{
}

AmlipIdDataType::AmlipIdDataType(
        const std::string& name)
    : AmlipIdDataType(name.c_str())
{
}

AmlipIdDataType::AmlipIdDataType(
        const char* name)
    : name_(char_name_to_array_(name))
    , rand_id_(random_id_())
{
}

AmlipIdDataType::AmlipIdDataType(
        const std::array<uint8_t, NAME_SIZE>& name,
        const std::array<uint8_t, RAND_SIZE>& rand_id)
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

bool AmlipIdDataType::operator <(
        const AmlipIdDataType& x) const
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
        return (rand_id_ < x.rand_id_);
    }
}

std::string AmlipIdDataType::name() const
{
    return array_name_to_str_(name_);
}

void AmlipIdDataType::name(
        const std::array<uint8_t, NAME_SIZE>& name)
{
    name_ = name;
}

void AmlipIdDataType::name(
        const std::string& name)
{
    name_ = str_name_to_array_(name);
}

std::array<uint8_t, NAME_SIZE> AmlipIdDataType::base64_name() const
{
    return name_;
}

std::array<uint8_t, RAND_SIZE> AmlipIdDataType::id() const
{
    return rand_id_;
}

std::array<uint8_t, RAND_SIZE>& AmlipIdDataType::id()
{
    return rand_id_;
}

void AmlipIdDataType::id(
        const std::array<uint8_t, RAND_SIZE>& id)
{
    rand_id_ = id;
}

std::string AmlipIdDataType::to_dds_string() const
{
    // NOTE: this uses to_string method as it will not use any invalid character for DDS
    return to_string();
}

std::string AmlipIdDataType::type_name()
{
    return TYPE_NAME_;
}

bool AmlipIdDataType::is_defined() const noexcept
{
    return (*this) != UNDEFINED_ID_;
}

std::string AmlipIdDataType::to_string() const noexcept
{
    // WARNING: If this method changes, it may change as well operator << and to_dds_string
    std::stringstream new_os;
    new_os << name();

    // Set to print bytes in hexadecimal of size 2 filling with 0
    new_os << std::hex << std::setfill('0');
    // TODO: use a common function that takes an array and format id without the final .
    for (uint8_t v : id())
    {
        new_os << "." << std::setw(2) << static_cast<unsigned>(v);
    }
    return new_os.str();
}

AmlipIdDataType AmlipIdDataType::new_unique_id()
{
    return AmlipIdDataType(random_name_(), random_id_());
}

AmlipIdDataType AmlipIdDataType::new_unique_id(
        const std::string& name)
{
    return new_unique_id(name.c_str());
}

AmlipIdDataType AmlipIdDataType::new_unique_id(
        const char* name)
{
    return AmlipIdDataType(char_name_to_array_(name), random_id_());
}

AmlipIdDataType AmlipIdDataType::undefined_id()
{
    return UNDEFINED_ID_;
}

std::array<uint8_t, NAME_SIZE> AmlipIdDataType::char_name_to_array_(
        const char* name)
{
    std::array<uint8_t, NAME_SIZE> _name = {0};
    size_t idx = 0;
    while (name[idx] != '\0' && idx < NAME_SIZE)
    {
        _name[idx] = name[idx];
        idx++;
    }

    return _name;
}

std::array<uint8_t, NAME_SIZE> AmlipIdDataType::str_name_to_array_(
        const std::string& name)
{
    return char_name_to_array_(name.c_str());
}

std::string AmlipIdDataType::array_name_to_str_(
        const std::array<uint8_t, NAME_SIZE>& name)
{
    auto it = std::find(name.begin(), name.end(), '\0');
    if (it != name.end())
    {
        return std::string(std::begin(name), it);
    }
    else
    {
        return std::string(std::begin(name), std::end(name));
    }
}

std::array<uint8_t, NAME_SIZE> AmlipIdDataType::random_name_()
{
    // make sure a random seed is properly set in the main scope

    std::array<uint8_t, NAME_SIZE> name;
    for (uint32_t i = 0; i < NAME_SIZE; i++)
    {
        // generate a random char between 'a' and 'z'
        char c = 'a' + static_cast<char>(rand() % 26);
        name[i] = static_cast<uint8_t>(c);
    }

    return name;
}

std::array<uint8_t, RAND_SIZE> AmlipIdDataType::random_id_()
{
    auto random_manager = utils::GlobalRandomManager::get_instance();
    std::array<uint8_t, RAND_SIZE> rand_id;
    for (uint32_t i = 0; i < RAND_SIZE; i++)
    {
        // generate a random number between 0 and 255
        uint8_t num = static_cast<uint8_t>(random_manager->pure_rand() % 256);
        rand_id[i] = num;
    }

    return rand_id;
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
    return new (memory) AmlipIdDataType();
}

std::ostream& operator <<(
        std::ostream& os,
        const AmlipIdDataType& id)
{
    // TODO do this from utils using container_to_stream to remove final '.'
    os << "ID{" << id.to_string() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include auxiliary functions like for serializing/deserializing.
#include  <types/id/impl/AmlipIdDataTypeCdrAux.ipp>
