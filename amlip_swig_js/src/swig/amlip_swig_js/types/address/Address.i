// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
// Binding for class Address
////////////////////////////////////////////////////////

// Any macro used on the header files will give an error if it is not redefined here
#define DDSPIPE_PARTICIPANTS_DllAPI

%{
#include <ddspipe_participants/types/address/Address.hpp>
%}

// Ignore overloaded methods that have no application on Python
%ignore eprosima::ddspipe::participants::types::operator <<(std::ostream &,const Address&);

// Include the class interfaces
%include <ddspipe_participants/types/address/Address.hpp>
