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

from amlip_py.types.AmlipIdDataType import AmlipIdDataType

from amlip_swig import NodeKind_agent, NodeKind_computing, NodeKind_discovery, NodeKind_main
from amlip_swig import NodeKind_meta, NodeKind_status, NodeKind_undetermined
from amlip_swig import StateKind_dropped, StateKind_running, StateKind_stopped, StateKind_unknown
from amlip_swig import StatusDataType as cpp_StatusDataType


class NodeKind(Enum):
    """TODO"""
    UNDETERMINED = NodeKind_undetermined
    DISCOVERY = NodeKind_discovery
    AGENT = NodeKind_agent
    MAIN = NodeKind_main
    COMPUTATIONAL = NodeKind_computing
    EDGE = NodeKind_status
    STATUS = NodeKind_meta


class StatusKind(Enum):
    """TODO"""
    UNKNOWN = StateKind_unknown
    RUNNING = StateKind_running
    STOPPED = StateKind_stopped
    DROPPED = StateKind_dropped


class StatusDataType(cpp_StatusDataType):
    """
    TODO
    """

    def __init__(self, *args):
        """TODO"""
        super().__init__(*args)

    def get_id(self) -> AmlipIdDataType:
        """TODO"""
        return cpp_StatusDataType.id(self)

    def get_node_kind(self) -> NodeKind:
        """TODO"""
        return cpp_StatusDataType.node_kind(self)

    def get_status_kind(self) -> StatusKind:
        """TODO"""
        return cpp_StatusDataType.status_kind(self)

    # TODO check if this is needed or it is taken from parent
    # def __str__(self) -> str:
    #     return super().__str__()
