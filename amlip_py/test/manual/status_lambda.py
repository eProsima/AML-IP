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

import signal

from amlip_py.node.StatusNode import StatusNode

# Domain ID
DOMAIN_ID = 166


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Status Node Py execution. Creating Node...')
    status_node = StatusNode('PyTestLambdaStatusNode', domain=DOMAIN_ID)

    # Launch node
    print(f'Node {status_node.id()} created. Processing data asynchronously...')
    status_node.process_status_async(
        callback=lambda status_received: print(
            f'Received status {status_received} by node {status_node.get_id()}'))

    # Expect for signal to arrive
    print('Already processing status data. Waiting SIGINT (C^)...')

    def handler(signum, frame):
        pass
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    print('SIGINT received. Finishing Manual Test Status Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
