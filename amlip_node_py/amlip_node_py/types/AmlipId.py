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
"""AML-IP AmlipId data type API specification."""

from amlip_swig import AmlipId as cpp_AmlipId
from amlip_swig import ID_SIZE


class AmlipId(cpp_AmlipId):
    """
    TODO
    """

    def __init__(self, *args):
        super().__init__(*args)

    def set_id(self, new_id: str) -> None:

        if new_id > ID_SIZE:
            raise Exception(f'Max number of chars for AMLIP id is {ID_SIZE}')
        else:
            return cpp_AmlipId.id(self, new_id)

    def get_id(self) -> str:
        return ''.join(cpp_AmlipId.id(self))

    def is_defined(self) -> bool:
        return cpp_AmlipId.is_defined(self)

    @staticmethod
    def new_unique_id() -> 'AmlipId':
        return cpp_AmlipId.new_unique_id()

    @staticmethod
    def undefined_id() -> 'AmlipId':
        return cpp_AmlipId.undefined_id()

    # def __str__(self) -> str:
    #     return self.get_id()
