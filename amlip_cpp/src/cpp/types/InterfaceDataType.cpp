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

#include <cpp_utils/exception/InconsistencyException.hpp>

#include <amlip_cpp/types/InterfaceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

bool InterfaceDataType::is_bounded()
{
    throw eprosima::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement is_bounded.");
}

bool InterfaceDataType::is_plain()
{
    throw eprosima::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement is_plain.");
}

bool InterfaceDataType::construct_sample(
        void*)
{
    throw eprosima::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement construct_sample.");
}

std::string InterfaceDataType::type_name()
{
    throw eprosima::utils::InconsistencyException(
              "Subclasses of InterfaceDataType must implement type_name.");
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
