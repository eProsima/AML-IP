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
// Binding for class StatusNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_cpp") "amlip_cpp/node/ParentNode.hpp";

// Generate directors for the virtual methods in the listener
// IMPORTANT: this statement must be before including the hpp
%feature("director") eprosima::amlip::node::StatusFunctor;

// Ignore operator () as it will be renamed with __call__ and is not accepted in python
%ignore eprosima::amlip::node::StatusFunctor::operator ()(const types::StatusDataType&) const;

// Declare the operator() method to use as __call__ in python
%extend eprosima::amlip::node::StatusFunctor {
    bool eprosima::amlip::node::StatusFunctor::__call__(const eprosima::amlip::types::StatusDataType& status) const
    {
        return (*$self).operator()(status);
    }
}

%{
#include <amlip_cpp/node/StatusNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/StatusNode.hpp>
