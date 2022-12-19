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
// Binding for class MainNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/node/ParentNode.hpp";

%{
#include <amlip_cpp/node/MainNode.hpp>
%}

%include "typemaps.i"

%typemap(in) (char* data, size_t size) {
    Py_ssize_t len;
    if (PyBytes_AsStringAndSize($input, &$1, &len) == -1)
        return NULL;
    $2 = (size_t)len;
}

%ignore request_job_solution(const types::JobDataType &);
%ignore request_job_solution(const types::JobDataType &, types::AmlipIdDataType &);

// Include the class interfaces
%include <amlip_cpp/node/MainNode.hpp>
