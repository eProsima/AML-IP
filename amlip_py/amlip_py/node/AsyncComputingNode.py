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

from amlip_swig import AsyncComputingNode as cpp_AsyncComputingNode
from amlip_swig import JobReplier as cpp_JobReplier


class JobReplier(cpp_JobReplier):
    """
    TODO
    """

    def process_job(
            self,
            job,
            task_id,
            client_id):
        """
        Raise exception.
        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('JobReplier.process_job must be specialized before use.')


class JobReplierLambda(cpp_JobReplier):
    """
    Custom JobReplier supporting to create it with a lambda function.
    This object is created with a lambda function that is stored inside and used for every
    JobSolution message received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def process_job(
            self,
            job,
            task_id,
            client_id):
        """Call internal lambda."""
        return self.callback_(job, task_id, client_id)


class AsyncComputingNode(cpp_AsyncComputingNode):
    """
    AML-IP Asynchronous Computing Node.
    TODO
    """

    def __init__(
            self,
            name: str,
            domain: int = None,
            callback=None,
            listener: JobReplier = None):
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
                'AsyncComputingNode constructor expects a listener object or a callback, '
                'both given.')

        if listener:
            self.listener_ = listener

        elif callback:
            self.listener_ = JobReplierLambda(callback)

        else:
            raise ValueError(
                'AsyncComputingNode constructor expects a listener object or a callback, '
                'none given.')

        # Parent class constructor
        if domain is None:
            super().__init__(name, self.listener_)
        else:
            super().__init__(name, self.listener_, domain)

    def run(
            self,
            ) -> None:
        """
        TODO
        """
        return cpp_AsyncComputingNode.run(self)

    def stop(
            self,
            ) -> None:
        """
        TODO
        """
        return cpp_AsyncComputingNode.stop(self)

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_AsyncComputingNode.id(self)
