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

from py_utils.wait.IntWaitHandler import IntWaitHandler

from amlip_py.node.AsyncComputingNode import AsyncComputingNode
from amlip_py.node.AsyncMainNode import AsyncMainNode
from amlip_py.types.JobDataType import JobDataType
from amlip_py.types.JobSolutionDataType import JobSolutionDataType


TEST_JOB_DATA = ['Irene', 'Javier', 'Juan', 'Raul']


def test_one_main_one_computing_async():
    """
    Create 1 Main and 1 Computing Node and send N jobs from one to the other.
    """
    # Create Waiter to wait for all solutions
    waiter = IntWaitHandler(True)

    # Store jobs sent indexed by task id
    jobs_sent = {}

    # Solution receive callback
    def solution_reception_callback(solution, task_id, _):
        assert solution.to_string() == jobs_sent[task_id].to_string().upper()
        waiter.increase()

    # Solution generation callback
    def job_reception_callback(job, task_id, client_id):
        return JobSolutionDataType(job.to_string().upper())

    # Create nodes
    main_node = AsyncMainNode(
        'AsyncMainNodePyTest',
        callback=solution_reception_callback)

    computing_node = AsyncComputingNode(
        'ComputingNodePyTest',
        callback=job_reception_callback)
    computing_node.run()

    for job_data in TEST_JOB_DATA:

        # Sent main job
        job = JobDataType(job_data)
        task_id = main_node.request_job_solution(job)
        jobs_sent[task_id] = job

    waiter.wait_equal(len(TEST_JOB_DATA))

    # Let everything finish kindly
