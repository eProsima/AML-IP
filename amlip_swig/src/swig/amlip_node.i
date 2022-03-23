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

%module amlip_node

// SWIG helper modules
%include "stdint.i"
%include "std_list.i"
%include "std_string.i"

// amlip_node
%include "amlip_node/StatusAmlipNode.i"
// amlip_types
%include "amlip_types/AmlipId.i"
%include "amlip_types/InterfaceDataType.i"
%include "amlip_types/Status.i"
