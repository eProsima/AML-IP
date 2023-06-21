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
import signal

from amlip_py.node.AsyncInferenceNode import AsyncInferenceNode, InferenceReplierLambda
from amlip_py.types.InferenceSolutionDataType import InferenceSolutionDataType

import numpy as np

from object_detection.utils import label_map_util

import tensorflow_hub as hub

# Domain ID
DOMAIN_ID = 166

# Not take into account detections with less probability than tolerance
tolerance = 25

current_path = os.path.abspath(__file__)
# Initialise model
path = current_path.split('amlip_tensorflow_inference_rosbot2r_demo', -1)[0]\
                        + 'amlip_tensorflow_inference_rosbot2r_demo/resource/\
tensorflow/models/centernet_hourglass_512x512_kpts_1'
dataset = current_path.split('amlip_tensorflow_inference_rosbot2r_demo', -1)[0]\
                           + 'amlip_tensorflow_inference_rosbot2r_demo/resource/\
tensorflow/models/research/object_detection/data/mscoco_label_map.pbtxt'

print('Model Handle at TensorFlow Hub: {}'.format(path))
print('loading model...')
hub_model = hub.load(path)
print('model loaded!')


def process_inference(
        inference,
        task_id,
        client_id):
    # Size | Image
    height, width = (inference.to_string().split(' | ', 1)[0]).split()
    image_str = inference.to_string().split(' | ', 1)[1]
    # Convert string to bytes
    img_bytes = base64.b64decode(image_str)
    # Convert bytes to image
    image = np.frombuffer((img_bytes), dtype=np.uint8).reshape((int(width), int(height), 3))
    string_inference = ''
    image_np = np.array(image).reshape((1, int(width), int(height), 3))
    results = hub_model(image_np)
    result = {key: value.numpy() for key, value in results.items()}
    category_index = label_map_util.create_category_index_from_labelmap(dataset,
                                                                        use_display_name=True)
    classes = (result['detection_classes'][0]).astype(int)
    scores = result['detection_scores'][0]
    for i in range(result['detection_boxes'][0].shape[0]):
        if (round(100*scores[i]) > tolerance):
            boxes = result['detection_boxes'][0]
            box = tuple(boxes[i].tolist())
            ymin, xmin, ymax, xmax = box
            string_inference = string_inference + \
                'Box [({}, {}), ({}, {})] {}: {}% \n' \
                .format(xmin, ymin, xmax, ymax, category_index[classes[i]]['name'],
                        round(100*scores[i]))
    print('Inference ready!')
    print('sending inference: ' + string_inference)
    return InferenceSolutionDataType(string_inference)


def main():
    # Create Node
    node = AsyncInferenceNode(
        'AMLInferenceNode',
        listener=InferenceReplierLambda(process_inference),
        domain=DOMAIN_ID)

    print(f'Inference Node {node.id()} ready. Waiting SIGINT (C^)...')

    node.run()

    def handler(signum, frame):
        pass
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    node.stop()

    print(f'Inference Node {node.id()} closing.')


# Call main in program execution
if __name__ == '__main__':
    main()
