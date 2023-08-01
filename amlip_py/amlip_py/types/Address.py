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
"""AML-IP Address data type API specification."""


from amlip_swig import Address as cpp_Address


class Address(cpp_Address):
    """TODO"""

    def __init__(
            self,
            ip: str = None,
            port: int = None,
            external_port: int = None,
            ip_version: int = None,
            domain: str = None,
            transport_protocol = None):

        if ip and ip_version:
            super().__init__(ip, port, external_port, ip_version, transport_protocol)
        elif ip_version:
            super().__init__(port, external_port, ip_version, domain, transport_protocol)
        elif domain:
            super().__init__(port, external_port, domain, transport_protocol)
        elif ip:
            super().__init__(ip, port, external_port, transport_protocol)
        else:
            super().__init__()
