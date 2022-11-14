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
"""AML-IP Status Node API specification."""

from amlip_swig import StatusNode as cpp_StatusNode
from amlip_swig import StatusNodeFunctor

from amlip_py.types.StatusDataType import StatusDataType


class CustomStatusNodeFunctor(StatusNodeFunctor):
    """Custom StatusNodeFunctor."""

    def __init__(self, callback):
        """Construct new object with lambda as callback."""
        self.callback_ = callback
        super().__init__()

    def __call__(self, status):
        """Call lambda."""
        self.callback_(status)
        return True


class StatusNode(cpp_StatusNode):
    """
    AML-IP Status Node.

    This Node implements a Reader to read every Status message published in the AML
    network, and prints it.
    """

    def __init__(self, lambda_callback=None):
        """
        Create a new StatusNode.

        :param lambda_callback: Callback to be called when a new Status is received.
        If lambda is given, use it whenever a new status is read.
        If it is not given (or None) use the default callback set in C++.
        """
        # Object attribute to store functor.
        # This cannot be removed while the callback could be called.
        self.custom_lambda_ = CustomStatusNodeFunctor(lambda_callback)

        # It is different a None than not passing an argument, so this if is needed
        if lambda_callback is None:
            super().__init__()
        else:
            # custom_lambda_ = CustomStatusNodeFunctor(lambda_callback)
            super().__init__(self.custom_lambda_)

    def spin(self) -> None:
        """
        Run this entity behavior till stop.

        Let this entity Reader read messages published in Status topic
        and show every of them in stdout till method stop is called.
        """
        return cpp_StatusNode.spin(self)

    def stop(self) -> None:
        """Stop this entity if it is spinning. Does nothing otherwise."""
        return cpp_StatusNode.stop(self)

    def get_id(self) -> StatusDataType:
        """Get id of the node."""
        return StatusDataType(cpp_StatusNode.id(self))
