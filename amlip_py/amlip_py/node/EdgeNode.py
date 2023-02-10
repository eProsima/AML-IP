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
"""AML-IP Edge Node API specification."""


from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.InferenceDataType import InferenceDataType

from amlip_swig import EdgeNode as cpp_EdgeNode


class EdgeNode(cpp_EdgeNode):
    """
    AML-IP Edge Node.

    TODO
    """

    def __init__(
            self,
            name: str):
        """
        Create a new Edge Node with a given name.

        Parameters
        ----------
        name : str
            Name of the node.
        """
        #####
        # Parent class constructor
        super().__init__(name)

    def request_inference(
            self,
            data: InferenceDataType):
        """
        Send a inference to a InferenceNode to be executed, and wait for the response.

        Parameters
        ----------
        data : StringInferenceDataType
            Inference that will be send to process by a inference node.

        Return
        ------
        Tuple[InferenceSolutionDataType, AmlipIdDataType]
        1. Solution to the inference given
        2. Id of the server
        """
        server_id = AmlipIdDataType()
        solution = cpp_EdgeNode.request_inference(self, data, server_id)
        return solution, server_id

    def get_id(
            self) -> AmlipIdDataType:
        """Get AMLIP id of the node."""
        return cpp_EdgeNode.id(self)
