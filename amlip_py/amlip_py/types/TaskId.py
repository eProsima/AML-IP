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
"""AML-IP TaskId data type API specification."""

from amlip_swig import TaskId as cpp_TaskId


class TaskId(cpp_TaskId):
    """Object that represents a Unique Id for each Task sent by a client."""

    def __init__(
            self,
            task_id: int = None):
        if task_id:
            super().__init__(task_id)
        else:
            super().__init__()
