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
import time

from py_utils.wait.IntWaitHandler import IntWaitHandler
from py_utils.wait.WaitHandler import AwakeReason

from amlip_py.node.AsyncMainNode import AsyncMainNode
from amlip_py.types.JobDataType import JobDataType


DESCRIPTION = """Script to execute an Asynchronous Main Node"""
USAGE = ('python3 main_node.py '
         '[-d <domain>] [-s <sleep>] [-n <name>] [<job1> <job2> ...]')


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
        '-s',
        '--sleep',
        type=float,
        default=0.1,
        help='Time to sleep after task sent.'
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
        default=['Default JOB 1', 'Default JOB 2'],
        help='Jobs to send in string format.'
    )

    return parser.parse_args()


def main():

    # Parse arguments
    args = parse_options()

    domain = args.domain
    sleep = args.sleep
    node_name = args.name
    jobs = args.args
    n_jobs = len(jobs)

    # Create Wait Handler so process waits for all solutions
    waiter = IntWaitHandler(True)

    # Prepare callback to call when solution received
    def process_solution_received(solution, task_id, server_id):
        print(
            f'Main Node {node_name} received solution for task <{task_id}> '
            f'from server <{server_id}> :\n'
            f'  <{solution.to_string()}>.',
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
        job_str = jobs.pop(0)

        # Send task and awaits for the solution
        task_id = node.request_job_solution(JobDataType(job_str))
        print(
            f'Main Node {node.id()} sent task <{job_str}> with task id <{task_id}>.',
            end='\n\n')

        # Wait for a bit
        time.sleep(sleep)

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
            f'Main Node {node.id()} closed with {n_jobs-waiter.get_value()} solutions remaining. Closing.',
            end='\n\n')

    elif reason == AwakeReason.condition_met:
        print(
            f'Main Node {node.id()} received all solutions. Closing.',
            end='\n\n')


# Call main in program execution
if __name__ == '__main__':
    main()
