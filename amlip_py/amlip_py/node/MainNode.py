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
"""AML-IP Main Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.JobDataType import JobDataType
from amlip_py.types.SolutionDataType import SolutionDataType

from amlip_swig import MainNode as cpp_MainNode


class MainNode(cpp_MainNode):
    """
    AML-IP Main Node.

    TODO
    """

    def __init__(
            self,
            name: str):
        """
        Create a new Main Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        """
        #####
        # Parent class constructor
        super().__init__(name)

    def request_job_solution(
            self,
            data: JobDataType) -> SolutionDataType:
        """
        Send a job to a ComputingNode to be executed, and wait for the response.

        Parameters
        ----------
        data : StringJobDataType
            Job that will be send to process by a computing node.

        Return
        ------
        Solution to the job given
        """
        return cpp_MainNode.request_job_solution(self, data)

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_MainNode.id(self)
