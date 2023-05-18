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
"""AML-IP Computing Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.JobDataType import JobDataType
from amlip_py.types.JobSolutionDataType import JobSolutionDataType

from amlip_swig import ComputingNode as cpp_ComputingNode
from amlip_swig import JobListener as cpp_JobListener


class JobListener(cpp_JobListener):
    """
    Job Listener class.

    This object must execute process_job method with each Job message that is received
    from node and must return the solution to the job.
    """

    def process_job(
            self,
            job: JobDataType) -> JobSolutionDataType:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('JobListener.Job_received must be specialized before use.')


class JobLambda(cpp_JobListener):
    """
    Custom JobListener supporting to create it with a lambda function.

    This object is created with a lambda function that is stored inside and used for every
    Job message received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def process_job(
            self,
            job: JobDataType) -> JobSolutionDataType:
        """Call internal lambda."""
        return self.callback_(job)


class ComputingNode(cpp_ComputingNode):
    """
    AML-IP Computing Node.

    TODO
    """

    def __init__(
            self,
            name: str,
            domain: int = None):
        """
        Create a new Computing Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        """
        #####
        # Parent class constructor
        if domain:
            super().__init__(name, domain)
        else:
            super().__init__(name)

    def process_job(
            self,
            callback=None,
            listener: JobListener = None) -> AmlipIdDataType:
        """
        Start processing the Job data arrived by using a lambda.

        This method supports 2 ways of including the lambda that describes what to do
        with each Job data that arrives. One is by a listener that may specialize method
        process_job and the other is retrieving a lambda as callback.

        Preconditions
        -------------
            One and only one of this arguments must be set

        Parameters
        ----------
        callback : lambda (StatusDataType)
            Lambda which only argument is a Status
            [Default = None]

        listener : StatusListener
            Object that specializes process_job method to execute in message reception.
            [Default = None]

        Return
        ------
        AmlipIdDataType : Id of the client sending this request
        """
        # If both arguments are set or neither of them, fail
        if (listener and callback):
            raise ValueError(
                'Method process_job expects a listener object or a callback, both given.')

        elif (not listener and not callback):
            raise ValueError(
                'Method process_job expects a listener object or a callback, none given.')

        client_id = AmlipIdDataType()
        # In case one and only one argument given, store it internally as a StatusListener
        # so this object is not destroyed while used.
        if listener:
            cpp_ComputingNode.process_job(self, listener, client_id)
        elif callback:
            cpp_ComputingNode.process_job(self, JobLambda(callback), client_id)

        return client_id

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_ComputingNode.id(self)
