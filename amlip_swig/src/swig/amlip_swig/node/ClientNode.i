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
// Binding for class ClientNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/node/wan/AgentNode.hpp";

%ignore eprosima::amlip::node::agent::ClientNode::get_router_configuration_();

namespace std {
   %template(addresses) set<eprosima::ddspipe::participants::types::Address>;
}

%{
#include <amlip_cpp/node/wan/ClientNode.hpp>

using namespace eprosima;
%}

// Include the class interfaces
%include <amlip_cpp/node/wan/ClientNode.hpp>