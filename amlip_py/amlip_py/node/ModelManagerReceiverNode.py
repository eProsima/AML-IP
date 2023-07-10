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
"""AML-IP Model Receiver Sender Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelDataType import ModelDataType
from amlip_py.types.ModelSolutionDataType import ModelSolutionDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

from amlip_swig import ModelManagerReceiverNode as cpp_ModelManagerReceiverNode
from amlip_swig import ModelListener as cpp_ModelListener


class ModelListener(cpp_ModelListener):
    """
    Model Listener class.

    This object must execute statistics_received method with each ModelStatisticsDataType that is received
    from node and must return true or false. This object must also execute model_received method on each
    ModelSolutionDataType that is received from node.
    """

    def statistics_received(
            self,
            statistics: ModelStatisticsDataType) -> bool:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('ModelListener.statistics_received must be specialized before use.')

    def model_received(
            self,
            model: ModelSolutionDataType) -> None:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('ModelListener.model_received must be specialized before use.')


class ModelListenerLambda(cpp_ModelListener):
    """
    Custom ModelListener supporting to create it with a lambda function.

    This object is created with a lambda function that is stored inside and used for every
    Model received.
    """

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def statistics_received(
            self,
            statistics: ModelStatisticsDataType) -> bool:
        """Call internal lambda."""
        return self.callback_(statistics)

    def ModelSolutionDataType(
            self,
            model: ModelSolutionDataType) -> None:
        """Call internal lambda."""
        return self.callback_(model)


class ModelManagerReceiverNode(cpp_ModelManagerReceiverNode):
    """
    AML-IP Model Manager Receiver Node.

    TODO
    """

    def __init__(
            self,
            id: AmlipIdDataType | str,
            data: ModelDataType,
            domain: int = None):

        """
        Create a new Model Manager Receiver Node with a given id.
        Parameters
        ----------
        id : AmlipIdDataType | str
        data: ModelDataType
        domain : int
        """

        # Parent class constructor
        if domain is None:
            super().__init__(id, data)
        else:
            super().__init__(id, data, domain)

    def start(
            self,
            callback = None,
            listener: ModelListener = None):

        # Set listener by one given or creating one for callback
        self.listener_ = None
        if listener and callback:
            raise ValueError(
                'ModelManagerReceiverNode constructor expects a listener object or a callback, '
                'both given.')

        if listener:
            self.listener_ = listener

        elif callback:
            self.listener_ = ModelListenerLambda(callback)

        else:
            raise ValueError(
                'ModelManagerReceiverNode constructor expects a listener object or a callback, '
                'none given.')

        return cpp_ModelManagerReceiverNode.start(self, self.listener_)

    def stop(
            self,
            ) -> None:

        """Stop this entity if it is running. Do nothing otherwise."""
        return cpp_ModelManagerReceiverNode.stop(self)
