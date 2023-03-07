// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
// Binding for class InferenceSolutionDataType
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/types/GenericDataType.hpp";

%include "std_vector.i"
namespace std {
   %template(bytes) vector<uint8_t>;
}

// Assignemt operators are ignored, as there is no such thing in Python.
// Trying to export them issues a warning
%ignore *::operator=;

%ignore eprosima::amlip::types::InferenceSolutionDataType::InferenceSolutionDataType();

%{
#include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>