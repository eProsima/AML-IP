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
import pickle

from py_utils.debugging.debug_utils import debug_variable_introspection
from py_utils.logging.log_utils import logging
from py_utils.wait.IntWaitHandler import IntWaitHandler
from py_utils.wait.WaitHandler import AwakeReason

from amlip_ctypes.amlip_ctypes import PyGenericDataType

from amlip_py.node.AsyncMainNode import AsyncMainNode

import datatypes


DESCRIPTION = """Script to execute an Asynchronous Main Node"""
USAGE = ('python3 main_node.py '
         '[-d <domain>] [-n <name>] [<file1> <file2> ...]')


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
        default='AML-MainNode',
        help='Node name.'
    )
    parser.add_argument(
        'args',
        nargs='*',
        default=[
            'resources/test.txt',
            'resources/caperucita.txt',
            'resources/lorem_ipsum.txt',
            'resources/lazarillo_de_tormes.txt',
            'resources/don_quijote_de_la_mancha.txt',
        ],
        help='File name to read and send job.'
    )

    return parser.parse_args()


def main():

    # Parse arguments
    args = parse_options()

    domain = args.domain
    node_name = args.name
    jobs = args.args
    n_jobs = len(jobs)
    tasks = {}

    # Create Wait Handler so process waits for all solutions
    waiter = IntWaitHandler(True)

    # Prepare callback to call when solution received
    def process_solution_received(solution, task_id, server_id):
        filename = tasks[task_id]

        print (f' === RECEIVED SOLUTION for {filename}')
        print (f'{solution}')
        # debug_variable_introspection(solution, debug_level=logging.INFO)
        # debug_variable_introspection(generic_type_view(solution), debug_level=logging.INFO)

        statistics = pickle.loads(
            PyGenericDataType(
                obj=solution))
        print(
            f'Main Node {node_name} received solution for task <{task_id}>:<{filename}> '
            f'from server <{server_id}> :\n'
            f'  <{statistics}>.',
            end='\n\n')
        waiter.increase()

    # Create Node
    node = AsyncMainNode(
        node_name,
        callback=process_solution_received,
        domain=domain)
    print(
        f'Main Node {node.id()} ready.',
        end='\n\n')

    # Iterate while arguments do not run out
    while jobs:

        # If arguments given, pop first one currently available
        filename = jobs.pop(0)

        # Send task and awaits for the solution
        job = PyGenericDataType(
                data=datatypes.Book(filename).tobytes()).to_dds()
        debug_variable_introspection(job, debug_level=logging.INFO)

        task_id = node.request_job_solution(job)
        print(
            f'Main Node {node.id()} sent task <{filename}> with task id <{task_id}>.',
            end='\n\n')
        tasks[task_id] = filename

    # Waiting for solutions
    print(
        f'Main Node {node.id()} waiting for solutions or SIGINT (C^).',
        end='\n\n')

    # If signal arrive, disable waiter
    def handler(signum, frame):
        print(
            ' Signal received, stopping.',
            end='\n\n')
        waiter.disable()
    signal.signal(signal.SIGINT, handler)

    reason = waiter.wait_greater_equal(n_jobs)

    # Closing
    if reason == AwakeReason.disabled:
        print(
            f'Main Node {node.id()} closed with '
            f'{n_jobs-waiter.get_value()} solutions remaining. Closing.',
            end='\n\n')

    elif reason == AwakeReason.condition_met:
        print(
            f'Main Node {node.id()} received all solutions. Closing.',
            end='\n\n')


# Call main in program execution
if __name__ == '__main__':
    main()
