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

import base64
import os

from amlip_py.node.AsyncEdgeNode import AsyncEdgeNode, InferenceListenerLambda
from amlip_py.types.InferenceDataType import InferenceDataType

import cv2

from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

# Variable to wait to the inference
waiter = BooleanWaitHandler(True, False)

# Domain ID
DOMAIN_ID = 166


def inference_received(
        inference,
        task_id,
        server_id):
    print(f'Edge Node received inference from {server_id}')
    print(f'Edge Node received inference {inference.to_string()}')
    waiter.open()


def main():
    # Create Node
    node = AsyncEdgeNode(
        'AMLAsyncEdgeNode',
        listener=InferenceListenerLambda(inference_received),
        domain=DOMAIN_ID)

    print(f'Async Edge Node {node.id()} ready.')

    # Image to inferred
    current_path = os.path.abspath(__file__)
    image_path = current_path.split('amlip_tensorflow_inference_demo', -1)[0]\
        + 'amlip_tensorflow_inference_demo/resource/tensorflow/models/research\
/object_detection/test_images/dog.jpg'
    img = cv2.imread(image_path)
    width = img.shape[1]
    height = img.shape[0]

    # Convert size to bytes
    str_size = str(width) + ' ' + str(height) + ' | '
    bytes_size = bytes(str_size, 'utf-8')
    # Convert image to bytes
    img_bytes = base64.b64encode(img)
    # Size + images
    img_size_bytes = bytes_size + img_bytes

    print(f'Edge Node {node.id()} sending data.')

    task_id = node.request_inference(InferenceDataType(img_size_bytes))

    print(f'Request sent with task id: {task_id}. Waiting inference...')

    # Wait to received solution
    waiter.wait()

    # Closing
    print(f'Edge Node {node.id()} closing.')


# Call main in program execution
if __name__ == '__main__':
    main()
