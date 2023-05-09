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

import sys

from amlip_py.node.MainNode import MainNode
from amlip_py.types.JobDataType import JobDataType


def main():
    # Create Node
    node = MainNode('AMLMainNode')
    print(f'Main Node {node.id()} ready.')

    has_input_args = (len(sys.argv) > 1)
    input_args = sys.argv[1:]

    # Iterate while user does not introduce empty string
    while True:

        job_str = ''

        if has_input_args:
            # Get string from argument input
            if input_args:
                job_str = input_args.pop(0)
            else:
                break

        else:
            # Get input from keyboard
            job_str = input('Please enter a string to create a job. Press enter to finish:\n> ')
            if not job_str:
                break

        # Send task and awaits for the solution
        print(f'Main Node {node.id()} sending task <{job_str}>.')
        solution, server_id = node.request_job_solution(JobDataType(job_str))
        print(f'Main Node received solution from {server_id}'
              f' for job <{job_str}> => <{solution.to_string()}>.')

    # Closing
    print(f'Main Node {node.id()} closing.')


# Call main in program execution
if __name__ == '__main__':
    main()
