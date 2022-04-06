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
"""AML-IP Generic Node API specification."""

from amlip_swig import GenericAmlipNode as cpp_GenericAmlipNode

class GenericAmlipNode(cpp_GenericAmlipNode):
    """
    AML-IP Generic Node.

    TODO
    """

    def __init__(self):
        super().__init__()

    def publish(self, data) -> None:
        return cpp_GenericAmlipNode.publish_vec(self, data)

    def receive(self) -> bytes:
        return cpp_GenericAmlipNode.receive_dump(self)

    def spin(self) -> None:
        return cpp_GenericAmlipNode.spin(self)

    def stop(self) -> None:
        """Stop this entity if it is spinning. Does nothing otherwise."""
        return cpp_GenericAmlipNode.stop(self)
