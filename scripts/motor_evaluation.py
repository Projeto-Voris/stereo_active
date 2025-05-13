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


class MotorEvalNode(Node):
    def __init__(self):
        super().__init__('motor_eval_node')

        # Parameters declaration
        self.declare_parameter('num_images', 30)
        self.declare_parameter('yaml_path', '~/ros2_ws/src/stereo_active/config/SM3.yaml')
        self.declare_parameter('motor_step', 1)

        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.yaml_file = self.get_parameter('yaml_path').get_parameter_value().string_value
        self.motor_step = self.get_parameter('motor_step').get_parameter_value().integer_value

        self.images_path = './{}_step{}'.format(time.strftime("%Y%m%d"), int(self.motor_step))
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

        self.motor_angle_pub = self.create_publisher(Float32, 'motor/angle', 10)


        # Create mutually exclusive callback groups
        self.callback_group_srv = MutuallyExclusiveCallbackGroup()
        self.callback_group_trigger_client = MutuallyExclusiveCallbackGroup()
        self.callback_group_laser_client = MutuallyExclusiveCallbackGroup()

        # Create the service from node
        self.srv = self.create_service(SetBool, 'acquire', self.get_images_srv, callback_group=self.callback_group_srv)
        self.gpio_client = self.create_client(Trigger, 'trigger', callback_group=self.callback_group_trigger_client)
        self.laser_client = self.create_client(SetBool, 'laser', callback_group=self.callback_group_laser_client)
        self.save_srv = self.create_service(Trigger, 'save', self.save_cb)

        self.count = 1
        self.perform_correl = False
        self.service_requet = False

    
    def save_cb(self, request, response):
        """
        Service callback to view the point cloud
        """
        self.images_path = './{}_step{}'.format(time.strftime("%Y%m%d_%H%m"), int(self.motor_step))

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
        self.motor_step = self.get_parameter('motor_step').get_parameter_value().integer_value
        self.service_requet = request.data

        self.count = 1
        self.left_images, self.right_images = [], []

        # Call laser service
        laser_request = SetBool.Request()
        laser_request.data = True  # Turn on the laser
        future_laser = self.laser_client.call_async(laser_request)
        rclpy.spin_until_future_complete(self, future_laser)

        # If laser service was successful, trigger the camera
        if future_laser.result() is not None:
            self.get_logger().info('Laser turned on')
            for n in range(self.num_images+4):
                float_msg = Float32()
                float_msg.data = self.motor_step/1024*360  # Example value
                self.motor_angle_pub.publish(float_msg)
                time.sleep(1.0)

                trigger_request = Trigger.Request()
                future = self.gpio_client.call_async(trigger_request)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    time.sleep(0.15)
                else:
                    self.get_logger().error('Service call failed')

        # Call laser service to turn off
        time.sleep(0.4)
        laser_request = SetBool.Request()
        laser_request.data = False  # Turn off the laser
        future_laser = self.laser_client.call_async(laser_request)
        float_msg.data = -(self.num_images+4)*self.motor_step/1024*360 
        self.motor_angle_pub.publish(float_msg)
        rclpy.spin_until_future_complete(self, future_laser)
        if future_laser.result() is not None:
            self.get_logger().info('Laser turned off')

        rclpy.spin_until_future_complete(self, future_laser)
        response.success = True
        response.message = 'Images captured successfully'

        return response

  

def main(args=None):
    rclpy.init(args=args)
    node = MotorEvalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()