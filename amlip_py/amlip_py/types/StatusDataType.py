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
"""AML-IP Status data type API specification."""

from enum import Enum

from amlip_swig import Status as cpp_Status
from amlip_swig import UNDETERMINED, DISCOVERY, AGENT, MAIN, COMPUTATIONAL, EDGE, STATUS
from amlip_swig import UNKNOWN, RUNNING, DISABLED

from amlip_py.types.AmlipIdDataType import AmlipIdDataType


class NodeKind(Enum):
    UNDETERMINED = UNDETERMINED
    DISCOVERY = DISCOVERY
    AGENT = AGENT
    MAIN = MAIN
    COMPUTATIONAL = COMPUTATIONAL
    EDGE = EDGE
    STATUS = STATUS


class StatusKind(Enum):
    UNKNOWN = UNKNOWN
    RUNNING = RUNNING
    DISABLED = DISABLED


class Status(cpp_Status):
    """
    TODO
    """

    def __init__(self, *args):
        super().__init__(*args)

    def get_id(self) -> AmlipIdDataType:
        return cpp_Status.id(self)

    def get_node_kind(self) -> NodeKind:
        return cpp_Status.node_kind(self)

    def get_status_kind(self) -> StatusKind:
        return cpp_Status.status_kind(self)

    # TODO check if this is needed or it is taken from parent
    # def __str__(self) -> str:
    #     return super().__str__()
