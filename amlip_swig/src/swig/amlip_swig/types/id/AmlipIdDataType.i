// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

////////////////////////////////////////////////////////
// Binding for class AmlipIdDataType
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/types/InterfaceDataType.hpp";

// Ignore overloaded methods that have no application on Python
// Otherwise they will issue a warning
%ignore eprosima::amlip::types::AmlipIdDataType::AmlipIdDataType(AmlipIdDataType&&);
%ignore eprosima::amlip::types::AmlipIdDataType::AmlipIdDataType(std::array< uint8_t,28> &&,std::array< uint8_t,4 > &&);
%ignore eprosima::amlip::types::AmlipIdDataType::id();
%rename("%s") eprosima::amlip::types::AmlipIdDataType::id() const;

%{
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
%}

%template(name) std::array<uint8_t, 28>;
%template(id) std::array<uint8_t, 4>;

// Include the class interfaces
%include <amlip_cpp/types/id/AmlipIdDataType.hpp>
