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
// Binding for class StatusDataType
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/types/InterfaceDataType.hpp";

// Include the class interfaces
// %include <amlip_cpp/types/id/StatusDataType.hpp>

// Assignemt operators are ignored, as there is no such thing in Python.
// Trying to export them issues a warning
%ignore *::operator=;

// Ignore overloaded methods that have no application on Python
// Otherwise they will issue a warning
%ignore eprosima::amlip::types::StatusDataType::StatusDataType(StatusDataType&&);
%ignore eprosima::amlip::types::operator <<(std::ostream &,const StatusDataType&);

// Declare the to string method
%extend eprosima::amlip::types::StatusDataType {
    std::string __str__() const
    {
        return $self->to_string();
    }
}

%{
#include <amlip_cpp/types/status/StatusDataType.hpp>
%}

// Include needed headers
%include <amlip_cpp/types/status/StatusDataType.hpp>
%include <amlip_cpp/types/status/StatusDataTypev1.hpp>
