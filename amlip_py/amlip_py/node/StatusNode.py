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
"""AML-IP Status Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.StatusDataType import StatusDataType

from amlip_swig import StatusListener as cpp_StatusListener
from amlip_swig import StatusNode as cpp_StatusNode


class StatusListener(cpp_StatusListener):
    """
    Status Listener class.

    This object must execute status_received method with each Status message that is received
    from  StatusNode.
    Must be inherited from user in order to reimplement status_received method.
    """

    def status_received(
            self,
            status: StatusDataType):
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('StatusListener.status_received must be specialized before use.')


class StatusLambda(StatusListener):
    """
    Custom StatusListener supporting to create it with a lambda function.

    This object is created with a lambda function that is stored inside and used for every
    Status message received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def status_received(
            self,
            status: StatusDataType):
        """Call internal lambda."""
        self.callback_(status)


class StatusNode(cpp_StatusNode):
    """
    AML-IP Status Node.

    This Node implements a Reader to read every Status message published in the AML network.
    With each message it executes the listener method (or lambda) given to process_status_async.

    This Node has an internal thread that listen to messages and execute the callback.
    In order to start processing messages, use process_status_async.
    In order to stop processing messages, use stop_processing.
    """

    def __init__(
            self,
            name: str):
        """
        Create a new Status Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        """
        #####
        # Internal variables
        self.internal_listener_ = None

        #####
        # Parent class constructor
        super().__init__(name)

    def process_status_async(
            self,
            callback=None,
            listener: StatusListener = None) -> None:
        """
        Start processing the Status data arrived by using a lambda.

        This method supports 2 ways of including the lambda that describes what to do
        with each Status data that arrives. One is by a listener that may specialize method
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
            Object that specializes status_received method to execute in message reception.
            [Default = None]

        """
        # If both arguments are set or neither of them, fail
        if (listener and callback):
            raise ValueError(
                'Method process_status_async expects a listener object or a callback, both given.')

        # In case one and only one argument given, store it internally as a StatusListener
        # so this object is not destroyed while used.
        elif listener:
            self.internal_listener_ = listener
            return cpp_StatusNode.process_status_async(self, self.internal_listener_)

        elif callback:
            self.internal_listener_ = StatusLambda(callback)
            return cpp_StatusNode.process_status_async(self, self.internal_listener_)

        else:
            raise ValueError(
                'Method process_status_async expects a listener object or a callback, none given.')

    def stop_processing(
            self) -> None:
        """Stop this entity if it is processing data. Do nothing otherwise."""
        cpp_StatusNode.stop(self)

        # Unstore the internal Listener
        self.internal_listener_ = None

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_StatusNode.id(self)
