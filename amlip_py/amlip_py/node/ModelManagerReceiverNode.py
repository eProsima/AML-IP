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
"""AML-IP Model Manager Receiver Node API specification."""

from typing import Union

from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelReplyDataType import ModelReplyDataType
from amlip_py.types.ModelRequestDataType import ModelRequestDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

from amlip_swig import ModelManagerReceiverNode as cpp_ModelManagerReceiverNode
from amlip_swig import ModelListener as cpp_ModelListener


class ModelListener(cpp_ModelListener):
    """
    Model Listener class.

    This object must be called by the ModelManagerReceiverNode to execute
    the statistics_received method with each ModelStatisticsDataType that is
    received from node and must return true or false. Must also execute the
    model_received method on each ModelReplyDataType that is received from node.
    """

    def statistics_received(
            self,
            statistics: ModelStatisticsDataType) -> bool:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('ModelListener.statistics_received must be specialized'
                                  ' before use.')

    def model_received(
            self,
            model: ModelReplyDataType) -> bool:
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

    def __init__(
            self,
            callback_statistics,
            callback_model):
        """Construct new object with lambda as callback."""
        self.callback_statistics_ = callback_statistics
        self.callback_model_ = callback_model
        super().__init__()

    def statistics_received(
            self,
            statistics: ModelStatisticsDataType) -> bool:
        """Call internal lambda."""
        return self.callback_statistics_(statistics)

    def model_received(
            self,
            model: ModelReplyDataType) -> bool:
        """Call internal lambda."""
        return self.callback_model_(model)


class ModelManagerReceiverNode(cpp_ModelManagerReceiverNode):
    """
    AML-IP Model Manager Receiver Node.
    """

    def __init__(
            self,
            id: Union[AmlipIdDataType, str],
            data: ModelRequestDataType,
            domain: int = None):

        """
        Create a new Model Manager Receiver Node with a given id.
        Parameters
        ----------
        id : Union[AmlipIdDataType, str]
        data: ModelRequestDataType
        domain : int
        """

        # Parent class constructor
        if domain is None:
            super().__init__(id, data)
        else:
            super().__init__(id, data, domain)

    def start(
            self,
            callback_statistics=None,
            callback_model=None,
            listener: ModelListener = None):

        # Set listener by one given or creating one for callback
        self.listener_ = None
        if (listener and callback_statistics) or (listener and callback_model):
            raise ValueError(
                'ModelManagerReceiverNode constructor expects a listener object or a callback, '
                'both given.')

        if listener:
            self.listener_ = listener

        elif callback_statistics and callback_model:
            self.listener_ = ModelListenerLambda(callback_statistics, callback_model)

        else:
            raise ValueError(
                'ModelManagerReceiverNode constructor expects a listener object or a '
                'callback_statistics and a callback_model, none given.')

        cpp_ModelManagerReceiverNode.start(self, self.listener_)

    def stop(
            self,
            ) -> None:

        """Stop this entity if it is running. Do nothing otherwise."""
        cpp_ModelManagerReceiverNode.stop(self)

    def request_model(
            self) -> None:
        """Request the model to the Model Manager Sender."""
        cpp_ModelManagerReceiverNode.request_model(self)

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_ModelManagerReceiverNode.id(self)
