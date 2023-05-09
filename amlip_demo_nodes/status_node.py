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

import argparse
import signal

from amlip_py.node.StatusNode import StatusNode


DESCRIPTION = """Script to execute an Asynchronous Computing Node. (Close with C^)"""
USAGE = ('python3 status_node.py '
         '[-d <domain>] [-n <name>]')


def parse_options():
    """
    Parse arguments.

    :return: The arguments parsed.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        add_help=True,
        description=(DESCRIPTION),
        usage=(USAGE)
    )
    parser.add_argument(
        '-d',
        '--domain',
        type=int,
        default=0,
        help='DDS Domain ID.'
    )
    parser.add_argument(
        '-n',
        '--name',
        type=str,
        default='AML-StatusNode',
        help='Node name.'
    )

    return parser.parse_args()


def main():
    # Parse arguments
    args = parse_options()

    domain = args.domain
    node_name = args.name

    # Prepare callback to call when status data arrive
    def status_received(status):
        print(
            f'Status Node {node_name} received status: <{status}>',
            end='\n\n')

    # Create node
    node = StatusNode(node_name, domain=domain)
    print(
        f'Status Node {node.id()} ready.',
        end='\n\n')

    # Launch node
    node.process_status_async(callback=status_received)

    # Expect for signal to arrive
    print(
        f'Node {node.id()} already processing status data. Waiting SIGINT (C^)...',
        end='\n\n')

    def handler(signum, frame):
        print(
            ' Signal received, stopping.',
            end='\n\n')
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    # Closing
    print(
        f'Status Node {node.id()} closing.',
        end='\n\n')


# Call main in program execution
if __name__ == '__main__':
    main()
