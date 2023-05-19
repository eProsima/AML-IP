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

import threading

from amlip_py.node.ComputingNode import ComputingNode
from amlip_py.node.MainNode import MainNode
from amlip_py.types.JobDataType import JobDataType
from amlip_py.types.JobSolutionDataType import JobSolutionDataType


TEST_JOB_DATA = ['Irene', 'Javier', 'Juan', 'Raul']


def process_job_routine(job: JobDataType) -> JobSolutionDataType:
    """
    Process a job and give a solution.

    In this test the process of the Computing Node is to uppercase a string.
    """
    return JobSolutionDataType(job.to_string().upper())


def test_one_main_one_computing():
    """
    Create 1 Main and 1 Computing Node and send N jobs from one to the other.

    The
    """
    # Create nodes
    main_node = MainNode('MainNodePyTest')
    computing_node = ComputingNode('ComputingNodePyTest')

    for job_data in TEST_JOB_DATA:
        # Process computing routine in a new thread
        computing_thread = threading.Thread(
            target=ComputingNode.process_job,
            args=(computing_node, process_job_routine))
        computing_thread.start()

        # Sent main job
        job = JobDataType(job_data)
        solution, server_id = main_node.request_job_solution(job)
        assert solution.to_string() == job_data.upper()
        assert server_id == computing_node.get_id()

        computing_thread.join()

    # Let everything finish kindly
