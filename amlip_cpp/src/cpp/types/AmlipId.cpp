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
 * @file AmlipId.cpp
 */

#include <memory>
#include <string>

#include <types/AmlipId.hpp>

namespace eprosima {
namespace amlip {
namespace types {

const std::string AmlipId::UNDEFINED_NAME_= "Undefined";

AmlipId::AmlipId(const std::string& name)
{
    data_ = std::make_shared<AmlipIdDataType>(name);
}

AmlipId::AmlipId(
        const AmlipId& x)
{
    data_ = std::make_shared<AmlipIdDataType>(*x.data_);
}

AmlipId::AmlipId(
        AmlipId&& x)
{
    data_ = std::move(x.data_);
}

AmlipId::~AmlipId()
{
}

AmlipId& AmlipId::operator =(
    const AmlipId& x)
{
    data_ = x.data_;

    return *this;
}

AmlipId& AmlipId::operator =(
    AmlipId&& x)
{
    data_ = std::move(x.data_);

    return *this;
}

bool AmlipId::operator ==(
    const AmlipId& x) const
{
    return *data_ == *x.data_;
}

bool AmlipId::operator !=(
        const AmlipId& x) const
{
    return !(*this == x);
}

std::string AmlipId::name() const
{
    return data_->name();
}

std::shared_ptr<AmlipIdDataType> AmlipId::data() const
{
    return data_;
}

AmlipId AmlipId::new_unique_id(const std::string& name)
{
    return AmlipId(name);
}

std::ostream& operator <<(
        std::ostream& os,
        const AmlipId& id)
{
    return os << *id.data();
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
