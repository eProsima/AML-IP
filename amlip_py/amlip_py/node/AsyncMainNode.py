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
"""AML-IP Async Main Node API specification."""

from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.JobDataType import JobDataType
from amlip_py.types.JobSolutionDataType import JobSolutionDataType

from amlip_swig import AsyncMainNode as cpp_AsyncMainNode
from amlip_swig import SolutionListener as cpp_SolutionListener


class SolutionListener(cpp_SolutionListener):
    """
    Solution Listener class.
    This object must execute solution_received method with each Solution message that is received
    from node and must return the solution to the job.
    """

    def solution_received(
            self,
            solution: JobSolutionDataType,
            task_id,
            server_id):
        """
        Raise exception.
        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('SolutionListener.solution_received must be specialized before use.')


class SolutionListenerLambda(SolutionListener):
    """
    Custom SolutionListener supporting to create it with a lambda function.
    This object is created with a lambda function that is stored inside and used for every
    JobSolution message received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def solution_received(
            self,
            solution: JobSolutionDataType,
            task_id,
            server_id):
        """Call internal lambda."""
        return self.callback_(solution, task_id, server_id)


class AsyncMainNode(cpp_AsyncMainNode):
    """
    AML-IP Asynchronous Main Node.
    TODO
    """

    def __init__(
            self,
            name: str,
            domain: int = None,
            callback=None,
            listener: SolutionListener = None):
        """
        Create a new Async Main Node with a given name.
        Parameters
        ----------
        name : str
            Name of the node.
        """
        # Set listener by one given or creating one for callback
        self.listener_ = None
        if listener and callback:
            raise ValueError(
                'AsyncMainNode constructor expects a listener object or a callback, both given.')

        elif listener:
            self.listener_ = listener

        elif callback:
            self.listener_ = SolutionListenerLambda(callback)

        else:
            raise ValueError(
                'AsyncMainNode constructor expects a listener object or a callback, none given.')

        # Parent class constructor
        if domain is None:
            super().__init__(name, self.listener_)
        else:
            super().__init__(name, self.listener_, domain)

    def request_job_solution(
            self,
            data: JobDataType):
        """
        Send a job to a ComputingNode to be executed, without actively waiting (async).
        Parameters
        ----------
        data : StringJobDataType
            Job that will be send to process by a computing node.
        Return
        ------
        Task id associated to the job given
        """
        return cpp_AsyncMainNode.request_job_solution(self, data)

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_AsyncMainNode.id(self)
