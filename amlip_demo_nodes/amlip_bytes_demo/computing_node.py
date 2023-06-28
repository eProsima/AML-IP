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

import argparse
import signal

from amlip_ctypes.amlip_ctypes import PyGenericDataType

from amlip_py.node.AsyncComputingNode import AsyncComputingNode

import datatypes


DESCRIPTION = """Script to execute an Asynchronous Computing Node. (Close with C^)"""
USAGE = ('python3 computing_node.py '
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
        default='AML-ComputingNode',
        help='Node name.'
    )

    return parser.parse_args()


def main():
    # Parse arguments
    args = parse_options()

    domain = args.domain
    node_name = args.name

    # Prepare callback to call when new job received
    def process_job(job, task_id, client_id):

        print(
            f'Computing Node {node_name} preparing solution for task <{task_id}> '
            f'from client <{client_id}>.\n')
        solution = datatypes.FileStatistics(job.to_string())
        print(
            f'  Solution : <{solution}>.',
            end='\n\n')
        return PyGenericDataType(
            data=solution.tobytes()).to_dds()

    # Create node
    node = AsyncComputingNode(
        node_name,
        callback=process_job,
        domain=domain)
    print(
        f'Computing Node {node.id()} ready.',
        end='\n\n')

    # Run
    node.run()
    print(
        f'Node {node.id()} already processing jobs. Waiting SIGINT (C^)...',
        end='\n\n')

    # Wait for signal to stop
    def handler(signum, frame):
        print(
            ' Signal received, stopping.',
            end='\n\n')
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    # Stopping node
    node.stop()

    # Closing
    print(
        f'Computing Node {node.id()} closing.',
        end='\n\n')


# Call main in program execution
if __name__ == '__main__':
    main()
