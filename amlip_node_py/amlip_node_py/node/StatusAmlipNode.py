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

from amlip_node import StatusAmlipNode as cpp_StatusAmlipNode


class StatusAmlipNode(cpp_StatusAmlipNode):
    """
    AML-IP Status Node.

    This Node implements a Reader to read every Status message published in the AML
    network, and prints it.
    """

    def __init__(self):
        """Create a new StatusAmlipNode."""
        super().__init__()

    def spin(self) -> None:
        """
        Run this entity behavior till stop.

        Let this entity Reader read messages published in Status topic
        and show every of them in stdout till method stop is called.
        """
        print('DEBUG: spinning py')
        return cpp_StatusAmlipNode.spin(self)

    def stop(self) -> None:
        """Stop this entity if it is spinning. Does nothing otherwise."""
        return cpp_StatusAmlipNode.stop(self)
