# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""AML-IP Model data type API specification."""


from amlip_swig import ModelDataType as cpp_ModelDataType


class ModelDataType(cpp_ModelDataType):
    """
    Object that represents a Model (request) sent from a ModelManagerReceiver Node to a ModelManagerSender one.
    """

    def __init__(
            self,
            message: (str | bytes) = None):
        """
        Construct a new Model request with name.
        Parameters
        ----------
        message: str or bytes
            Data to send to ModelManagerSender Node.
        """
        if (message):
            super().__init__(message)
        else:
            super().__init__()

    def __str__(
            self) -> str:
        """Serialize into a string."""
        return cpp_ModelDataType.to_string(self)

    def to_string(
            self) -> str:
        """Serialize into a string."""
        return cpp_ModelDataType.to_string(self)
