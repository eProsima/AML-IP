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

# Types
import amlip_py.types.AmlipIdDataType
import amlip_py.types.JobDataType
import amlip_py.types.JobSolutionDataType
import amlip_py.types.StatusDataType
import amlip_py.types.TaskId
import amlip_py.types.ModelRequestDataType
import amlip_py.types.ModelReplyDataType
import amlip_py.types.ModelStatisticsDataType
import amlip_py.types.Address


def test_id():
    """Test creation of AmlipIdDataType."""
    _ = amlip_py.types.AmlipIdDataType.AmlipIdDataType()


def test_status():
    """Test creation of StatusDataType."""
    _ = amlip_py.types.StatusDataType.StatusDataType()


def test_job():
    """Test creation of JobDataType."""
    _ = amlip_py.types.JobDataType.JobDataType()


def test_solution():
    """Test creation of JobSolutionDataType."""
    _ = amlip_py.types.JobSolutionDataType.JobSolutionDataType()


def test_task():
    """Test creation of TaskId."""
    _ = amlip_py.types.TaskId.TaskId()


def test_model():
    """Test creation of ModelRequestDataType."""
    _ = amlip_py.types.ModelRequestDataType.ModelRequestDataType()


def test_model_solution():
    """Test creation of ModelReplyDataType."""
    _ = amlip_py.types.ModelReplyDataType.ModelReplyDataType()


def test_model_statistics():
    """Test creation of ModelStatisticsDataType."""
    _ = amlip_py.types.ModelStatisticsDataType.ModelStatisticsDataType()


def test_address():
    """Test creation of Address."""
    _ = amlip_py.types.Address.Address()
