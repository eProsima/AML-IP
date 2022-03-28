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

%module(threads="1") amlip_node

// SWIG helper modules
%include "stdint.i"
%include "std_array.i"
%include "std_list.i"
%include "std_string.i"
%include "std_shared_ptr.i"
%include "std_vector.i"

// amlip_types
%include "amlip_node/types/InterfaceDataType.i"
%include "amlip_node/types/AmlipId.i"
%include "amlip_node/types/Status.i"
// amlip_node
%include "amlip_node/node/StatusAmlipNode.i"
