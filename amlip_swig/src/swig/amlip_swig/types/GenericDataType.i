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

// Ignore overloaded methods that have no application on Python
// Otherwise they will issue a warning
%ignore eprosima::amlip::types::GenericDataType::GenericDataType(GenericDataType&&);
%ignore eprosima::amlip::types::GenericDataType::data_size();
%rename("%s") eprosima::amlip::types::GenericDataType::data_size() const;

// Declare the to string method
%extend eprosima::amlip::types::GenericDataType {
    std::string __str__() const
    {
        return $self->to_string();
    }
}

%{
#include <amlip_cpp/types/GenericDataType.hpp>
%}

%include <fastcdr/config.h>

// Include the class interfaces
%include <amlip_cpp/types/GenericDataType.hpp>
