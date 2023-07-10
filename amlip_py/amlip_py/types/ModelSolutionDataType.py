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
"""AML-IP Model Solution data type API specification."""


from amlip_swig import ModelSolutionDataType as cpp_ModelSolutionDataType


class ModelSolutionDataType(cpp_ModelSolutionDataType):
    """
    Object that represents a Model (reply) sent from a ModelManagerSender
    Node to a ModelManagerReceiver one.
    """

    def __init__(
            self,
            message: (str | bytes) = None):
        """
        Construct a new Model reply with name.
        Parameters
        ----------
        message: str or bytes
            Data to send to ModelManagerReceiver Node.
        """
        if (message):
            super().__init__(message)
        else:
            super().__init__()

    def __str__(
            self) -> str:
        """Serialize into a string."""
        return cpp_ModelSolutionDataType.to_string(self)

    def to_string(
            self) -> str:
        """Serialize into a string."""
        return cpp_ModelSolutionDataType.to_string(self)
