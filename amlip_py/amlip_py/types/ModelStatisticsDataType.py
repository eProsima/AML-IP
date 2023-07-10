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
"""AML-IP Model Statistics data type API specification."""


from amlip_swig import ModelStatisticsDataType as cpp_ModelStatisticsDataType
import AmlipIdDataType


class ModelStatisticsDataType(cpp_ModelStatisticsDataType):
    """
    Object that represents a Model (request) sent from a ModelManagerReceiver Node to a ModelManagerSender one.
    """

    def __init__(
            self,
            message: (str | bytes) = None):
        """
        Construct a new Model Statistics with name.
        Parameters
        ----------
        message: str or bytes
            Statistics to send to ModelManagerReceiver Node.
        """
        if (message):
            super().__init__(message)
        else:
            super().__init__()

    def get_name(self) -> str:
        """Get name referenced to this Id."""
        return cpp_ModelStatisticsDataType.name()

    def get_server_id(self) -> AmlipIdDataType:
        """Get server Id referenced to this Id."""
        return cpp_ModelStatisticsDataType.server_id()

    def get_data(self):
        """Get data referenced to this Id."""
        return cpp_ModelStatisticsDataType.data()
