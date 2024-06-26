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
"""AML-IP Solution data type API specification."""

from typing import Union

from amlip_swig import InferenceSolutionDataType as cpp_InferenceSolutionDataType


class InferenceSolutionDataType(cpp_InferenceSolutionDataType):
    """
    Object that represents an Inference sent from a Edge Node to a Inference one.
    """

    def __init__(
            self,
            message: Union[bytes, str] = None):
        """
        Construct a new Inference with name.
        Parameters
        ----------
        message: str or bytes
            Data to send to Inference Node serialized in a string of basic chars.
        """
        if (message):
            super().__init__(message)
        else:
            super().__init__()

    def __str__(
            self) -> str:
        """Serialize Inference into a string."""
        return cpp_InferenceSolutionDataType.to_string(self)

    def to_string(
            self) -> str:
        """Serialize Inference into a string."""
        return cpp_InferenceSolutionDataType.to_string(self)
