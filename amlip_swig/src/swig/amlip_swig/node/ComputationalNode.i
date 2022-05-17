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
// Binding for class ComputationalNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_cpp") "amlip_cpp/node/ParentNode.hpp";

// Generate directors for the virtual methods in the listener
// IMPORTANT: this statement must be before including the hpp
%feature("director") eprosima::amlip::node::JobFunctor;

// Ignore operator () as it will be renamed with __call__ and is not accepted in python
%ignore eprosima::amlip::node::JobFunctor::operator ()(const types::JobDataType&) const;

// Declare the operator() method to use as __call__ in python
%extend eprosima::amlip::node::JobFunctor {
    eprosima::amlip::types::SolutionDataType eprosima::amlip::node::JobFunctor::__call__(const eprosima::amlip::types::JobDataType& job) const
    {
        return (*$self).operator()(job);
    }
}

%{
#include <amlip_cpp/node/ComputationalNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/ComputationalNode.hpp>
