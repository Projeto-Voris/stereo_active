#!/usr/bin/env python3
import os
import cv2
import numpy as np
import cupy as cp
import time
import struct

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from std_msgs.msg import Header
import message_filters
from std_srvs.srv import Trigger, SetBool


class ProjectEvalNode(Node):
    def __init__(self):
        super().__init__('project_eval_node')
        self.declare_parameter('num_images', 30)

        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value


        self.images_path = './{}'.format(time.strftime("%Y%m%d"))
        self.get_logger().info(f'Number of images to be captured: {self.num_images}')


        self.bridge = CvBridge()

        self.left_images = []
        self.right_images = []

        # Initialize the subscribers
        self.left_image_sub = message_filters.Subscriber(self, Image, 'left/image')
        self.right_image_sub = message_filters.Subscriber(self, Image, 'right/image')


        # Synchronize the stereo images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub],
                                                    queue_size=10, slop=0.05)
        self.ts.registerCallback(self.stereo_images_callback)



        # Create mutually exclusive callback groups
        self.callback_group_srv = MutuallyExclusiveCallbackGroup()
        self.callback_group_trigger_client = MutuallyExclusiveCallbackGroup()
        self.callback_group_laser_client = MutuallyExclusiveCallbackGroup()

        # Create the service from node
        self.srv = self.create_service(SetBool, 'acquire_pattern', self.get_images_srv, callback_group=self.callback_group_srv)
        self.gpio_client = self.create_client(Trigger, 'trigger', callback_group=self.callback_group_trigger_client)
        self.project_pattern = self.create_client(Trigger, 'next_image', callback_group=self.callback_group_laser_client)
        self.save_srv = self.create_service(Trigger, 'save_pattern', self.save_cb)

        self.count = 1
        self.perform_correl = False
        self.service_requet = False

    
    def save_cb(self, request, response):
        """
        Service callback to view the point cloud
        """
        self.images_path = './{}_pattern_images'.format(time.strftime("%Y%m%d"))

        if request:
            self.get_logger().info('Saving images')
            os.makedirs(self.images_path, exist_ok=True)
            os.makedirs(os.path.join(self.images_path, 'left'), exist_ok=True)
            os.makedirs(os.path.join(self.images_path, 'right'), exist_ok=True)
            n = 1
            for left, right in zip(self.left_images, self.right_images):
                cv2.imwrite(os.path.join(self.images_path,'left/L{:03d}.png').format(n), left)
                cv2.imwrite(os.path.join(self.images_path,'right/R{:03d}.png').format(n), right)
                n += 1
            if len(os.listdir(os.path.join(self.images_path, 'left'))) == self.num_images:
                response.success = True
                response.message = 'Images saved successfully'
                self.get_logger().info('Images saved')
            else:
                response.success = False
                response.message = 'Images not saved'
                self.get_logger().error('Images saved with some mistake')
        return response

    def stereo_images_callback(self, left_image, right_image):
        """
        Callback function for the stereo images subscriber
        """

        if self.count <= self.num_images:
            self.get_logger().info('Images callback received - {}'.format(self.count))
        else:
            # self.get_logger().info('Images received')
            return

        if left_image.encoding == 'bgr8' or right_image.encoding == 'bgr8':
            left_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
            right_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        elif left_image.encoding == 'mono8' or left_image.encoding == 'mono16':


            left_image = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='mono8')

            right_image = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='mono8')
            # cv2.imwrite('img_r.png', right_image)
            # cv2.imwrite('img_l.png', left_image)

        if self.service_requet:    
            self.left_images.append(left_image)
            self.right_images.append(right_image)
            self.count +=1

    def get_images_srv(self, request, response):
        """
        Service callback to get stereo images
        """
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.service_requet = True
        self.count = 1
        self.left_images, self.right_images = [], []


        for n in range(self.num_images):
            time.sleep(1.0)

            trigger_request = Trigger.Request()
            future = self.gpio_client.call_async(trigger_request)
            rclpy.spin_until_future_complete(self, future)
            next_image_request = Trigger.Request()
            
            if future.result() is not None:
                time.sleep(0.15)
                future_image = self.project_pattern.call_async(next_image_request)
                rclpy.spin_until_future_complete(self, future_image)
                if future_image.result() is not None:
                    time.sleep(0.15)
                    self.get_logger().info('Pattern projected')
                else:
                    self.get_logger().error('Service next_image call failed')
            else:
                self.get_logger().error('Service call failed')

        # Call laser service to turn off
        time.sleep(0.4)

        response.success = True
        response.message = 'Images captured successfully'

        return response

  

def main(args=None):
    rclpy.init(args=args)
    node = ProjectEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()