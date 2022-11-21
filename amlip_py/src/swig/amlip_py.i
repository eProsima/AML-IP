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

%exception {
    try { $action }
    catch (Swig::DirectorException &e) { SWIG_fail; }
}

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

// types
%include "amlip_py/types/GenericDataType.i"
%include "amlip_py/types/id/AmlipIdDataType.i"
%include "amlip_py/types/InterfaceDataType.i"
%include "amlip_py/types/job/JobDataType.i"
%include "amlip_py/types/job/SolutionDataType.i"
%include "amlip_py/types/multiservice/MsReferenceDataType.i"
%include "amlip_py/types/multiservice/MsRequestDataType.i"
%include "amlip_py/types/status/StatusDataType.i"
// node
%include "amlip_py/node/ComputationalNode.i"
%include "amlip_py/node/MainNode.i"
%include "amlip_py/node/ParentNode.i"
%include "amlip_py/node/StatusNode.i"
