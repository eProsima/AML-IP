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
import re
import time

from amlip_py.node.EdgeNode import EdgeNode
from amlip_py.types.InferenceDataType import InferenceDataType

from cv_bridge import CvBridge      # Package to convert between ROS and OpenCV Images

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import Image


class SubscriberImage(Node):

    def __init__(self):
        super().__init__('subscriber_image')
        custom_qos_profile = QoSProfile(
                            depth=1,
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


class PublisherVel(Node):

    def __init__(self):
        super().__init__('publisher_velocity')
        custom_qos_profile = QoSProfile(
                            history=QoSHistoryPolicy.KEEP_ALL,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            custom_qos_profile)

    def turn(self):
        msg = Twist()

        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -1.0

        self.pub.publish(msg)


def turn_rosbot():
    rclpy.init(args=None)
    node = PublisherVel()

    print('Turn ROSbot')
    loop = 0
    while rclpy.ok() and loop < 13:
        node.turn()
        time.sleep(0.5)
        loop += 1

    node.destroy_node()
    rclpy.shutdown()


def check_data(str_inference):
    labels = re.findall(r'\b(\w+):', str_inference)
    percentages = re.findall(r'(\d+)%', str_inference)
    print('Labels: ')
    print(labels)
    print('Percentages: ')
    print(percentages)
    for i in range(len(labels)):
        if (labels[i] == 'person' and int(percentages[i]) >= 80):
            print('Found a person!')
            turn_rosbot()
            return
    print('No person found :(')


def main():
    rclpy.init(args=None)

    minimal_subscriber = SubscriberImage()

    while rclpy.ok() and not minimal_subscriber.image_arrive:
        rclpy.spin_once(minimal_subscriber)

    img = minimal_subscriber.image

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    # Display image - debug
    # cv2.imshow("ROSbot2R Camera", img)

    # cv2.waitKey(0)
    # cv2.destroyWindow("ROSbot2R Camera")

    # Create Node
    node = EdgeNode('AMLEdgeNode')
    print(f'Edge Node {node.id()} ready.')

    # Image to inferred
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

    inference, server_id = node.request_inference(InferenceDataType(img_size_bytes))
    print(f'Edge Node received inference from {server_id}')

    print(f'Edge Node received inference {inference.to_string()}')

    check_data(inference.to_string())

    # Closing
    print(f'Edge Node {node.id()} closing.')


# Call main in program execution
if __name__ == '__main__':
    main()
