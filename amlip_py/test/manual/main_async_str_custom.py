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

from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

from amlip_py.node.AsyncMainNode import AsyncMainNode, SolutionListener
from amlip_py.types.JobDataType import JobDataType

# Maximum time to wait for solution
MAX_TIMEOUT = 10


class CustomSolutionListener(SolutionListener):

    def __init__(self, waiter):
        self.waiter_ = waiter
        super().__init__()

    def solution_received(
            self,
            solution,
            task_id,
            server_id):
        print(f'Solution received from server: {server_id}\n with id: {task_id}\n solution: {solution.to_string()}')
        self.waiter_.open()


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Variable to wait to the solution
    waiter = BooleanWaitHandler(True, False)

    # Create node
    print('Starting Manual Test Async Main Node Py execution. Creating Node...')
    listener = CustomSolutionListener(waiter)
    main_node = AsyncMainNode('PyTestAsyncMainNode', listener=listener)

    # Create job data
    print(f'Node created: {main_node.get_id()}. Creating job...')
    data_str = '<Job Data In Py String>'
    job_data = JobDataType(data_str)

    # Sending job
    print(f'Job data created with string: {job_data}. Sending request...')
    task_id = main_node.request_job_solution(job_data)

    print(f'Request sent with task id: {task_id}. Waiting solution...')

    # Wait to received solution
    waiter.wait()

    print('Finishing Manual Test Async Main Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
