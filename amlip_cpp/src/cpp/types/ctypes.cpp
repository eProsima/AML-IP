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

#include <amlip_cpp/types/ctypes.hpp>

eprosima::amlip::types::GenericDataType* ctypes_GenericDataType_new(void* data, const uint32_t size)
{
    return new eprosima::amlip::types::GenericDataType(data, size);
}

void* ctypes_GenericDataType_data(const eprosima::amlip::types::GenericDataType* d)
{
    return d->data();
}

uint32_t ctypes_GenericDataType_data_size(const eprosima::amlip::types::GenericDataType* d)
{
    return d->data_size();
}

eprosima::amlip::types::GenericDataType* ctypes_GenericDataType_from_ptr(void* p)
{
    return (eprosima::amlip::types::GenericDataType*) p;
}

std::shared_ptr<eprosima::amlip::types::GenericDataType> ctypes_GenericDataType_to_shared_ptr(eprosima::amlip::types::GenericDataType* d)
{
    return std::shared_ptr<eprosima::amlip::types::GenericDataType>(d);
}