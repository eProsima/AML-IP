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

////////////////////////////////////////////////////////
// Binding for class GenericDataType
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/types/InterfaceDataType.hpp";

namespace std {
   %template(bytes) vector<uint8_t>;
}

// Assignemt operators are ignored, as there is no such thing in Python.
// Trying to export them issues a warning
%ignore *::operator=;

// Ignore overloaded methods that have no application on Python
// Otherwise they will issue a warning
%ignore eprosima::amlip::types::GenericDataType::GenericDataType(GenericDataType&&);
%ignore eprosima::amlip::types::operator <<(std::ostream &,const GenericDataType&);

%{
#include <amlip_cpp/types/GenericDataType.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/types/GenericDataType.hpp>
