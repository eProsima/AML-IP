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

%module(directors="1", threads="1") amlip_swig_js

// Macro delcarations
// Any macro used on the header files will give an error if it is not redefined here
#define AMLIP_CPP_DllAPI

// SWIG helper modules
// %include "cpointer.i"
%include "stdint.i"
// %include "std_array.i"
// %include "std_list.i"
// %include "std_set.i"
// %include "std_string.i"
// %include "std_shared_ptr.i"
%include "std_vector.i"

// Definition of internal types
typedef short int16_t;
typedef int int32_t;
typedef long int64_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long uint64_t;

// IMPORTANT: the order of these includes is relevant, and must keep same order of cpp declarations.
// types
%include "amlip_swig_js/types/InterfaceDataType.i"
%include "amlip_swig_js/types/GenericDataType.i"
// %include "amlip_swig_js/types/id/AmlipIdDataType.i"
// %include "amlip_swig_js/types/id/TaskId.i"
// %include "amlip_swig_js/types/status/NodeKind.i"
// %include "amlip_swig_js/types/status/StateKind.i"
// %include "amlip_swig_js/types/status/StatusDataType.i"

// // node
// %include "amlip_swig_js/node/ParentNode.i"
// %include "amlip_swig_js/node/MainNode.i"
