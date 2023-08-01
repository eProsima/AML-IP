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
"""AML-IP Agent Client Node API specification."""

from amlip_py.types.Address import Address

from amlip_swig import ClientNode as cpp_ClientNode


class ClientNode(cpp_ClientNode):
    """
    AML-IP Agent Client Node.

    TODO
    """

    def __init__(
            self,
            name: str,
            connection_addresses: Address,
            domain: int = None):
        """
        Create a new Agent Client Node with a given name and a connection_addresses.

        Parameters
        ----------
        name : str
        connection_addresses : Address
        domain : int
        """
        #####
        # Parent class constructor
        if domain is None:
            super().__init__(name, connection_addresses)
        else:
            super().__init__(name, connection_addresses, domain)
