# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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


from amlip_swig import JobSolutionDataType as cpp_JobSolutionDataType


class JobSolutionDataType(cpp_JobSolutionDataType):
    """
    Object that represents a Job sent from a Main Node to a Computing one.

    TODO
    ----
    This class does only support string conversion.
    """

    def __init__(
            self,
            message: str = None):
        """TODO"""        """
        Construct a new Job with name.

        Parameters
        ----------
        message: str
            Data to send to Computing Node serialized in a string of basic chars.
        """
        if message:
            super().__init__(message)
        else:
            super().__init__()

    def __str__(
            self) -> str:
        """Serialize Job into a string."""
        return cpp_JobSolutionDataType.to_string(self)

    def to_string(
            self) -> str:
        """Serialize Job into a string."""
        return cpp_JobSolutionDataType.to_string(self)

    def view(self) -> memoryview:
        return generic_type_view(self)
