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

from amlip_py.node.EdgeNode import EdgeNode
from amlip_py.types.InferenceDataType import InferenceDataType


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Edge Node Py execution. Creating Node...')
    edge_node = EdgeNode('PyTestEdgeNode')

    # Create inference data
    print(f'Node created: {edge_node.get_id()}. Creating inference...')
    data_str = '<Inference Data In Py String>'
    inference_data = InferenceDataType(data_str)

    # Sending inference
    print(f'Inference data created with string: {inference_data}. Sending request...')
    solution, server_id = edge_node.request_inference(inference_data)

    # Deserializing solution
    print(f'Solution received from server {server_id}. Deserializing to string...')
    solution_str = solution.to_string()

    print(f'Solution deserialized is: {solution_str}. '
          'Finishing Manual Test Edge Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
