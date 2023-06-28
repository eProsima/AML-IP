
import ctypes
import ctypes.util

from py_utils.debugging.debug_utils import debug_variable_introspection
from py_utils.logging.log_utils import logging

from amlip_swig import ctypes_GenericDataType_to_shared_ptr, ctypes_GenericDataType_from_ptr



# For type checking the returned pointer.
class _GenericDataType(ctypes.c_void_p):
    pass
PGenericDataType = ctypes.POINTER(_GenericDataType)


class PyGenericDataType(ctypes.Structure):

    library_path = ctypes.util.find_library('amlip_cpp')
    amlip_cpp_library = ctypes.CDLL(library_path)  # Update with the correct path

    amlip_cpp_library.ctypes_GenericDataType_new.argtypes = (
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_size_t)
    amlip_cpp_library.ctypes_GenericDataType_new.restype = PGenericDataType

    amlip_cpp_library.ctypes_GenericDataType_data.argtypes = (PGenericDataType, )
    amlip_cpp_library.ctypes_GenericDataType_data.restype = ctypes.POINTER(ctypes.c_uint8)

    amlip_cpp_library.ctypes_GenericDataType_data_size.argtypes = (PGenericDataType, )
    amlip_cpp_library.ctypes_GenericDataType_data_size.restype = ctypes.c_size_t

    def __init__(
            self,
            data: (bytes|bytearray|memoryview) = None,
            obj=None):

        print(f'PyGenericDataType: {data} {len(data)}')

        if data is not None:
            self.size = len(data)
            array = ctypes.c_uint8 * self.size
            mv = memoryview(data)
            self.obj = self.amlip_cpp_library.ctypes_GenericDataType_new(
                array.from_buffer(mv),
                self.size)

        elif obj is not None:
            self.obj = obj
            self.size = self.amlip_cpp_library.ctypes_GenericDataType_data_size(obj)

        else:
            raise ValueError(
                'PyGenericDataType requires data or GenericDataType object')

    def to_dds(self):

        debug_variable_introspection(self.obj, debug_level=logging.INFO)

        v = ctypes.cast(self.obj, ctypes.POINTER(ctypes.c_void_p))
        debug_variable_introspection(v, debug_level=logging.INFO)

        p = ctypes_GenericDataType_from_ptr(self.obj)
        debug_variable_introspection(p, debug_level=logging.INFO)

        s = ctypes_GenericDataType_to_shared_ptr(p)
        debug_variable_introspection(s, debug_level=logging.INFO)

        return s

    def to_memoryview(self):
        return memoryview(
            ctypes.cast(
                self.amlip_cpp_library.ctypes_GenericDataType_data(self.obj).
                ctypes.POINTER(ctypes.c_uint8 * self.size)).contents)
