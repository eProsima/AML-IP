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

"""
 * @file status.py
"""

import signal

from amlip_py.node.StatusAmlipNode import StatusAmlipNode

def handler():
    # status_node.stop()
    print("SIGINT received. Destroying entities...")


def create_node():

    # Create Status Node
    status_node = StatusAmlipNode("TestStatusNode")

    print("Node created: " + status_node + ". Processing data asynchronously...")

    # Create callback that only prints by stdout the status that arrives
    status_node.spin()

    print( "Already processing status data. Waiting SIGINT (C^)...")

    signal.signal(signal.SIGINT, handler)




# initialize the random number generator ¿?

print("Starting Manual Test Status Node execution. Creating Node...")
create_node()
print("Finishing Manual Test Status Node execution.")
