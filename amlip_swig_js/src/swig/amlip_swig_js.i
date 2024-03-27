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

%module amlip_swig_js

// Macro delcarations
// Any macro used on the header files will give an error if it is not redefined here
#define AMLIP_CPP_DllAPI

// SWIG helper modules
%include "cpointer.i"
%include "stdint.i"
%include "std_string.i"
%include "std_vector.i"
%include "std_unique_ptr.i"

// IMPORTANT: the order of these includes is relevant, and must keep same order of cpp declarations.
// types
%include "amlip_swig_js/types/InterfaceDataType.i"
%include "amlip_swig_js/types/GenericDataType.i"
%include "amlip_swig_js/types/id/AmlipIdDataType.i"
%include "amlip_swig_js/types/id/TaskId.i"
%include "amlip_swig_js/types/status/NodeKind.i"
%include "amlip_swig_js/types/status/StateKind.i"
%include "amlip_swig_js/types/status/StatusDataType.i"
%include "amlip_swig_js/types/job/JobDataType.i"
%include "amlip_swig_js/types/job/JobSolutionDataType.i"
%include "amlip_swig_js/types/inference/InferenceDataType.i"
%include "amlip_swig_js/types/inference/InferenceSolutionDataType.i"
%include "amlip_swig_js/types/model/ModelRequestDataType.i"
%include "amlip_swig_js/types/model/ModelReplyDataType.i"
%include "amlip_swig_js/types/model/ModelStatisticsDataType.i"
%include "amlip_swig_js/types/address/Address.i"

// node
%include "amlip_swig_js/node/ParentNode.i"
%include "amlip_swig_js/node/StatusNode.i"
%include "amlip_swig_js/node/MainNode.i"
%include "amlip_swig_js/node/ComputingNode.i"
%include "amlip_swig_js/node/AsyncMainNode.i"
%include "amlip_swig_js/node/AsyncComputingNode.i"
%include "amlip_swig_js/node/EdgeNode.i"
%include "amlip_swig_js/node/AsyncEdgeNode.i"
%include "amlip_swig_js/node/InferenceNode.i"
%include "amlip_swig_js/node/AsyncInferenceNode.i"
%include "amlip_swig_js/node/ModelManagerSenderNode.i"
%include "amlip_swig_js/node/ModelManagerReceiverNode.i"
// TODO
// %include "amlip_swig_js/node/AgentNode.i"
// %include "amlip_swig_js/node/ClientNode.i"
// %include "amlip_swig_js/node/ServerNode.i"
// %include "amlip_swig_js/node/RepeaterNode.i"
