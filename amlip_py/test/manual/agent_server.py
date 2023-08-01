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

import signal

from amlip_py.node.ServerNode import ServerNode
from amlip_py.types.Address import Address
from amlip_swig import TransportProtocol_udp

# Domain ID
DOMAIN_ID = 166


def main():
    """Execute main routine."""

    # Create listening address
    listening_address = Address(
        port=12121,
        external_port=12121,
        domain='localhost',
        transport_protocol=TransportProtocol_udp)

    # Create node
    print('Starting Manual Test Agent Server Node Py execution. Creating Node...')

    ServerNode(
        name='PyTestServerNode',
        listening_addresses=[listening_address],
        domain=DOMAIN_ID)

    print('Node created. Waiting SIGINT (C^)...')

    def handler(signum, frame):
        pass
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    print('Finishing Manual Test Agent Server Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
