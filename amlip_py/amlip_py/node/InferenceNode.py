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
"""AML-IP Inference Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.InferenceDataType import InferenceDataType
from amlip_py.types.InferenceSolutionDataType import InferenceSolutionDataType

from amlip_swig import InferenceNode as cpp_InferenceNode
from amlip_swig import InferenceListener as cpp_InferenceListener


class InferenceListener(cpp_InferenceListener):
    """
    Inference Listener class.

    This object must execute process_inference method with each Inference message that is received
    from node and must return the solution to the inference.
    """

    def process_inference(
            self,
            inference: InferenceDataType) -> InferenceSolutionDataType:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError(
            'InferenceListener.Inference_received must be specialized before use.')


class InferenceLambda(InferenceListener):
    """
    Custom InferenceListener supporting to create it with a lambda function.

    This object is created with a lambda function that is stored inside and used for every
    Inference message received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def process_inference(
            self,
            inference: InferenceDataType) -> InferenceSolutionDataType:
        """Call internal lambda."""
        return self.callback_(inference)


class InferenceNode(cpp_InferenceNode):
    """
    AML-IP Inference Node.

    TODO
    """

    def __init__(
            self,
            name: str):
        """
        Create a new Inference Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        """
        #####
        # Parent class constructor
        super().__init__(name)

    def process_inference(
            self,
            callback=None,
            listener: InferenceListener = None) -> AmlipIdDataType:
        """
        Start processing the Inference data arrived by using a lambda.

        This method supports 2 ways of including the lambda that describes what to do
        with each Inference data that arrives. One is by a listener that may specialize method
        process_inference and the other is retrieving a lambda as callback.

        Preconditions
        -------------
            One and only one of this arguments must be set

        Parameters
        ----------
        callback : lambda (StatusDataType)
            Lambda which only argument is a Status
            [Default = None]

        listener : StatusListener
            Object that specializes process_inference method to execute in message reception.
            [Default = None]

        Return
        ------
        AmlipIdDataType : Id of the client sending this request
        """
        # If both arguments are set or neither of them, fail
        if (listener and callback):
            raise ValueError(
                'Method process_inference expects a listener object or a callback, both given.')

        elif (not listener and not callback):
            raise ValueError(
                'Method process_inference expects a listener object or a callback, none given.')

        client_id = AmlipIdDataType()
        # In case one and only one argument given, store it internally as a StatusListener
        # so this object is not destroyed while used.
        if listener:
            cpp_InferenceNode.process_inference(self, listener, client_id)
        elif callback:
            cpp_InferenceNode.process_inference(self, InferenceLambda(callback), client_id)

        return client_id

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_InferenceNode.id(self)
