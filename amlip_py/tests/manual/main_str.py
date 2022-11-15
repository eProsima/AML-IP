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

from amlip_py.node.MainNode import MainNode
from amlip_py.types.JobDataType import JobDataType


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Main Node Py execution. Creating Node...')
    main_node = MainNode('PyTestMainNode')

    # Create job data
    print(f'Node created: {main_node.get_id()}. Creating job...')
    data_str = '<Job Data In Py String>'
    job_data = JobDataType(data_str)

    # Sending job
    print(f'Job data created with string: {job_data}. Sending request...')
    solution = main_node.request_job_solution(job_data)

    # Deserializing solution
    print('Solution received. Deserializing to string...')
    solution_str = solution.to_string()

    print(f'Solution deserialized is: {solution_str}. '
          'Finishing Manual Test Main Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
