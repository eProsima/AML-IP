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
"""AML-IP AmlipIdDataType data type API specification."""


from amlip_swig import AmlipIdDataType as cpp_AmlipIdDataType


class AmlipIdDataType(cpp_AmlipIdDataType):
    """Object that represents a Unique Id for each AML-IP Node."""

    def __init__(
            self,
            name: str = None):
        """Construct a new Id with name if given."""
        if name:
            super().__init__(name)
        else:
            super().__init__()

    def get_name(self) -> str:
        """Get name referenced to this Id."""
        return cpp_AmlipIdDataType.name()

    def is_defined(self) -> bool:
        """Wether the Id is defined."""
        return cpp_AmlipIdDataType.is_defined(self)

    def set_id(self, id) -> bool:
        """Defines the Id."""
        return cpp_AmlipIdDataType.id(self, id)
