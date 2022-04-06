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
 * @file InterfaceDataType.cpp
 */

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <types/InterfaceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

size_t InterfaceDataType::get_max_cdr_serialize_size(
        size_t current_alignment)
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement get_max_cdr_serialize_size.");
}

size_t InterfaceDataType::get_cdr_serialize_size(
        const InterfaceDataType&,
        size_t current_alignment)
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement get_cdr_serialize_size.");
}

size_t InterfaceDataType::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement get_key_max_cdr_serialized_size.");
}

bool InterfaceDataType::is_key_defined()
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement is_key_defined.");
}

bool InterfaceDataType::is_bounded()
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement is_bounded.");
}

bool InterfaceDataType::is_plain()
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement is_plain.");
}

bool InterfaceDataType::construct_sample(
        void*)
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement construct_sample.");
}

const char* InterfaceDataType::type_name()
{
    throw ddsrouter::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement type_name.");
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
