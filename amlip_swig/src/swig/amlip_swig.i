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

%module(directors="1", threads="1") amlip_swig

// Handle exceptions on python callbacks and send them back to C++ so that they can be catched
// Also, add some meaningful description of the error
%feature("director:except") {
  if ($error != NULL) {
    PyObject *exc, *val, *tb;
    PyErr_Fetch(&exc, &val, &tb);
    PyErr_NormalizeException(&exc, &val, &tb);
    std::string err_msg("In method '$symname': ");

    PyObject* exc_str = PyObject_GetAttrString(exc, "__name__");
    err_msg += PyUnicode_AsUTF8(exc_str);
    Py_XDECREF(exc_str);

    if (val != NULL)
    {
      PyObject* val_str = PyObject_Str(val);
      err_msg += ": ";
      err_msg += PyUnicode_AsUTF8(val_str);
      Py_XDECREF(val_str);
    }

    Py_XDECREF(exc);
    Py_XDECREF(val);
    Py_XDECREF(tb);

    Swig::DirectorMethodException::raise(err_msg.c_str());
  }
}

// If using windows in debug, it would try to use python_d, which would not be found.
%begin %{
#ifdef _MSC_VER
#define SWIG_PYTHON_INTERPRETER_NO_DEBUG
#endif
#include <exception>
%}

// Macro delcarations
// Any macro used on the header files will give an error if it is not redefined here
#define AMLIP_CPP_DllAPI

// SWIG helper modules
%include "stdint.i"
%include "std_array.i"
%include "std_list.i"
%include "std_string.i"
%include "std_shared_ptr.i"
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
%include "amlip_swig/types/InterfaceDataType.i"
%include "amlip_swig/types/GenericDataType.i"
%include "amlip_swig/types/id/AmlipIdDataType.i"
%include "amlip_swig/types/status/NodeKind.i"
%include "amlip_swig/types/status/StateKind.i"
%include "amlip_swig/types/status/StatusDataType.i"
%include "amlip_swig/types/job/JobDataType.i"
%include "amlip_swig/types/job/JobSolutionDataType.i"
%include "amlip_swig/types/inference/InferenceDataType.i"
%include "amlip_swig/types/inference/InferenceSolutionDataType.i"
// node
%include "amlip_swig/node/ParentNode.i"
%include "amlip_swig/node/StatusNode.i"
%include "amlip_swig/node/MainNode.i"
%include "amlip_swig/node/ComputingNode.i"
%include "amlip_swig/node/EdgeNode.i"
