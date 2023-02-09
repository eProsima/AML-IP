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

import time

from amlip_py.node.ComputingNode import ComputingNode, JobLambda
from amlip_py.types.JobSolutionDataType import JobSolutionDataType


def main():
    """Execute Computing routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Computing Node Py execution. Creating Node...')
    computing_node = ComputingNode('PyTestComputingNode')

    # Create listener
    print(f'Node created: {computing_node.get_id()}. Creating Listener...')
    job_listener = JobLambda(
        lambda job: JobSolutionDataType(job.to_string().lower()))

    # Launch node
    print('Listener created. Processing job...')
    client_id = computing_node.process_job(listener=job_listener)

    print(f'Solution sent to client {client_id}. '
          'Finishing Manual Test Computing Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
