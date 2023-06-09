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

from amlip_py.node.InferenceNode import InferenceNode, InferenceLambda
from amlip_py.types.InferenceSolutionDataType import InferenceSolutionDataType


def main():
    """Execute Inference routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Inference Node Py execution. Creating Node...')
    inference_node = InferenceNode('PyTestInferenceNode')

    # Create listener
    print(f'Node created: {inference_node.get_id()}. Creating Listener...')
    inference_listener = InferenceLambda(
        lambda inference: InferenceSolutionDataType(inference.to_string().lower()))

    # Launch node
    print('Listener created. Processing inference...')
    client_id = inference_node.process_inference(listener=inference_listener)

    print(f'Solution sent to client {client_id}. '
          'Finishing Manual Test Inference Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
