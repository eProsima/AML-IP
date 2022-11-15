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


from amlip_swig import SolutionDataType as cpp_SolutionDataType


class SolutionDataType(cpp_SolutionDataType):
    """TODO"""

    def __init__(
            self,
            message: str):
        """TODO"""
        super().__init__(message)

    def __str__(
            self) -> str:
        """TODO"""
        return cpp_SolutionDataType.to_string(self)


    def to_string(
            self) -> str:
        """TODO"""
        return cpp_SolutionDataType.to_string(self)


    def to_vector(
            self) -> bytes:
        """TODO"""
        return cpp_SolutionDataType.to_vector(self)
