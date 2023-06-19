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

import os

import cv2
import base64
import time
import re
import pygame

from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

from amlip_py.node.AsyncEdgeNode import AsyncEdgeNode, InferenceListenerLambda
from amlip_py.types.InferenceDataType import InferenceDataType

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


# Variable to wait to the inference
waiter = BooleanWaitHandler(True, False)

# Domain ID
DOMAIN_ID = 166


class SubscriberImage(Node):

    def __init__(self):
        super().__init__('subscriber_image')
        custom_qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_ALL,
        reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            custom_qos_profile)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.image = None
        self.image_arrive = False

    def listener_callback(self, msg):
        self.get_logger().info('I received an image!!')
        # Convert ROS Image message to OpenCV image
        self.image = self.br.imgmsg_to_cv2(msg)
        self.image_arrive = True


def beep():
    pygame.mixer.init()
    current_path = os.path.abspath(__file__)
    beep_path = current_path.split("amlip_tensorflow_inference_demo",-1)[0]+"amlip_tensorflow_inference_demo/resource/beep.wav"
    beep_sound = pygame.mixer.Sound(beep_path)
    beep_sound.set_volume(1.0)
    beep_sound.play()
    time.sleep(1)
    beep_sound.stop()
    pygame.mixer.quit()

def check_data(str):
    labels = re.findall(r'\b(\w+):', str)
    percentages = re.findall(r'(\d+)%', str)
    print('Labels: ')
    print(labels)
    print('Percentages: ')
    print(percentages)
    for i in range(len(labels)):
        if(labels[i] == 'person' and int(percentages[i]) >= 80):
            print('Found a person!')
            beep()
            return
    print('No person found :(')

def inference_received(
        inference,
        task_id,
        server_id):
    print(f'Edge Node received inference from {server_id}')
    print(f"Edge Node received inference {inference.to_string()}")
    check_data(inference.to_string())
    waiter.open()


def main():
    rclpy.init(args=None)

    minimal_subscriber = SubscriberImage()

    while rclpy.ok() and not minimal_subscriber.image_arrive:
        rclpy.spin_once(minimal_subscriber)

    img = minimal_subscriber.image

    minimal_subscriber.destroy_node()
    rclpy.try_shutdown()

    # Display image
    cv2.imshow("ROSbot2R Camera", img)

    cv2.waitKey(0)
    cv2.destroyWindow("ROSbot2R Camera")

    # Create Node
    node = AsyncEdgeNode(
        'AMLAsyncEdgeNode',
        listener=InferenceListenerLambda(inference_received),
        domain=DOMAIN_ID)

    print(f'Async Edge Node {node.id()} ready.')

    width = img.shape[1]
    height = img.shape[0]

    # Convert size to bytes
    str_size = str(width) + " " + str(height) + " | "
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
