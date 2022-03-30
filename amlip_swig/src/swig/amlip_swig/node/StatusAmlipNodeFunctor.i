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

%{
#include <amlip_node/node/StatusAmlipNodeFunctor.hpp>
%}

// generate directors for the virtual methods in the listener
%feature("director") eprosima::amlip::node::StatusAmlipNodeFunctor;

%include <amlip_node/node/StatusAmlipNodeFunctor.hpp>

// Ignore the global comparison operators and make them class-internal
%ignore eprosima::amlip::node::StatusAmlipNodeFunctor::operator ()(eprosima::amlip::types::Status) const;

// Declare the to string method
%extend eprosima::amlip::node::StatusAmlipNodeFunctor {
    bool eprosima::amlip::node::StatusAmlipNodeFunctor::__call__(eprosima::amlip::types::Status status) const
    {
        return (*$self).operator()(status);
    }
}