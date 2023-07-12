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
"""AML-IP Model Manager Sender Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelDataType import ModelDataType
from amlip_py.types.ModelSolutionDataType import ModelSolutionDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

from amlip_swig import ModelManagerSenderNode as cpp_ModelManagerSenderNode
from amlip_swig import ModelReplier as cpp_ModelReplier


class ModelReplier(cpp_ModelReplier):
    """
    Model Replier class.

    This object must execute send_model method with each ModelDataType request that is received
    from node and must return the ModelSolutionDataType reply.
    """

    def send_model(
            self,
            model: ModelDataType) -> ModelSolutionDataType:
        """
        Raise exception.

        Abstract method.
        This method should be reimplemented by child class.
        """
        raise NotImplementedError('ModelReplier.send_model must be specialized before use.')


class ModelReplierLambda(cpp_ModelReplier):
    """
    Custom ModelReplier supporting to create it with a lambda function.

    This object is created with a lambda function that is stored inside and used for every
    ModelDataType message received.
    """

    def __init__(
            self,
            callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def send_model(
            self,
            model: ModelDataType) -> ModelSolutionDataType:
        """Call internal lambda."""
        return self.callback_(model)


class ModelManagerSenderNode(cpp_ModelManagerSenderNode):
    """
    AML-IP Model Manager Sender Node.
    """

    def __init__(
            self,
            id: AmlipIdDataType,
            statistics: ModelStatisticsDataType,
            domain: int = None):

        """
        Create a new Model Manager Sender Node with a given id.
        Parameters
        ----------
        id : AmlipIdDataType
        statistics : ModelStatisticsDataType
        domain : int
        """

        # Parent class constructor
        if domain is None:
            super().__init__(id, statistics)
        else:
            super().__init__(id, statistics, domain)

    def start(
            self,
            callback=None,
            listener: ModelReplier = None):

        # Set listener by one given or creating one for callback
        self.listener_ = None
        if listener and callback:
            raise ValueError(
                'ModelManagerSenderNode constructor expects a listener object or a callback, '
                'both given.')

        if listener:
            self.listener_ = listener

        elif callback:
            self.listener_ = ModelReplierLambda(callback)

        else:
            raise ValueError(
                'ModelManagerSenderNode constructor expects a listener object or a callback, '
                'none given.')

        return cpp_ModelManagerSenderNode.start(self, self.listener_)

    def stop(
            self) -> None:

        """Stop this entity if it is running. Do nothing otherwise."""
        return cpp_ModelManagerSenderNode.stop(self)

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_ModelManagerSenderNode.id(self)
