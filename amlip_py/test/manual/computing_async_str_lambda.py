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

from amlip_py.node.AsyncComputingNode import AsyncComputingNode
from amlip_py.types.JobSolutionDataType import JobSolutionDataType

# Domain ID
DOMAIN_ID = 10


def process_job(
        job,
        task_id,
        client_id):
    solution = JobSolutionDataType(job.to_string().lower())
    print(f'Job received from client: {client_id}\n'
          f' with id: {task_id}\n'
          f' job: {job.to_string()}\n'
          f' solution: {solution.to_string()}')
    return solution


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Async Computing Node Py execution. Creating Node...')
    computing_node = AsyncComputingNode(
        'PyTestAsyncComputingNode',
        callback=process_job,
        domain=DOMAIN_ID)

    # Create job data
    print(f'Node created: {computing_node.get_id()}. '
          'Already processing jobs. Waiting SIGINT (C^)...')

    computing_node.run()

    def handler(signum, frame):
        pass
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    computing_node.stop()

    print('Finishing Manual Test Async Computing Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
