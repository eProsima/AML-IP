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
"""AML-IP Agent Server Node API specification."""

from amlip_py.types.Address import Address

from amlip_swig import ServerNode as cpp_ServerNode


class ServerNode(cpp_ServerNode):
    """
    AML-IP Agent Server Node.

    TODO
    """

    def __init__(
            self,
            name: str,
            listening_addresses: Address,
            domain: int = None):
        """
        Create a new Agent Server Node with a given name and a listening_addresses.

        Parameters
        ----------
        name : str
        listening_addresses : Address
        domain : int
        """
        #####
        # Parent class constructor
        if domain is None:
            super().__init__(name, listening_addresses)
        else:
            super().__init__(name, listening_addresses, domain)
