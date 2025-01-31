#!/usr/bin/env python3
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from std_msgs.msg import Header
import message_filters
import time
from std_srvs.srv import Trigger, SetBool

from scripts.InverseTriangulation import InverseTriangulation

class InverseTriangulationNode(Node):
    def __init__(self):
        super().__init__('inverse_triangulation_node')
        self.get_logger().info('InverseTriangulationNode has been started.')

        self.declare_parameter('num_images', 10)
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.get_logger().info(f'Number of images to be captured: {self.num_images}')
        self.declare_parameter('yaml_path', '~/ros2_ws/src/stereo_active/config/SM3.yaml')
        self.yaml_file = self.get_parameter('yaml_path').get_parameter_value().string_value


        self.Zscan = InverseTriangulation(yaml_file=self.yaml_file)
        self.bridge = CvBridge()

        self.left_images = []
        self.right_images = []

        # self.create_subscription(CameraInfo, 'left/camera_info', self.camera_info_left_cb, 1)
        # self.create_subscription(CameraInfo, 'right/camera_info', self.camera_info_right_cb, 1)

        self.left_image_sub = message_filters.Subscriber(self, Image, 'left/image')
        self.right_image_sub = message_filters.Subscriber(self, Image, 'right/image')

        self.motor_angle_pub = self.create_publisher(Float32, 'motor/angle', 10)


        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub],
                                                    queue_size=10, slop=0.05)
        self.ts.registerCallback(self.stereo_images_callback)

        self.pcl_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # Create mutually exclusive callback groups
        self.callback_group_srv = MutuallyExclusiveCallbackGroup()
        self.callback_group_trigger_client = MutuallyExclusiveCallbackGroup()
        self.callback_group_laser_client = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(Trigger, 'get_images', self.get_images_srv, callback_group=self.callback_group_srv)
        self.gpio_client = self.create_client(Trigger, 'trigger', callback_group=self.callback_group_trigger_client)
        self.laser_client = self.create_client(SetBool, 'laser', callback_group=self.callback_group_laser_client)
        self.view_srv = self.create_service(Trigger, 'view', self.view_cb)

        self.count = 0
    
    def view_cb(self, request, response):
        """
        Service callback to view the point cloud
        """
        if request:
            self.get_logger().info('Viewing images')
            for n in range(len(self.left_images)):
                cv2.imwrite('L{:02d}.png'.format(n + 1), self.left_images[n])
                cv2.imwrite('R{:02d}.png'.format(n + 1), self.right_images[n])
            if len(self.left_images) == self.num_images:
                response.success = True
                response.message = 'Images saved successfully'
            else:
                response.success = False
                response.message = 'Images not saved'
        return response

    def stereo_images_callback(self, left_image, right_image):
        """
        Callback function for the stereo images subscriber
        """
        self.get_logger().info('Images callback received - {}'.format(self.count))

        if left_image.encoding == 'bgr8' or right_image.encoding == 'bgr8':
            left_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
            right_images = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        else:
            left_images = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='mono8')
            right_images = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='mono8')
    
        self.left_images.append(left_images)
        self.right_images.append(right_images)
        self.count +=1
        if self.count >= self.num_images:
            # self.correlation_process();
            self.get_logger().info('Images received')
            self.count = 0
            self.get_logger().info('Calling spatial correlation')
            # self.spatial_correl_process()

    def get_images_srv(self, request, response):
        """
        Service callback to get stereo images
        """
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        if request:
            self.count = 0
            self.left_images, self.right_images = [], []
            float_msg = Float32()
            float_msg.data = 400.0  # Example value
            self.motor_angle_pub.publish(float_msg)
            # Call laser service
            laser_request = SetBool.Request()
            laser_request.data = True  # Turn on the laser
            future_laser = self.laser_client.call_async(laser_request)
            rclpy.spin_until_future_complete(self, future_laser)
            # If laser service was successful, trigger the camera
            if future_laser.result() is not None:
                self.get_logger().info('Laser turned on')
                # rclpy.spin_once(self)
                # time.sleep(0.5)  # Delay to wait for the image to be captured
                for n in range(self.num_images):
                    # self.get_logger().info(f'Waiting for image {n+1}')
                    trigger_request = Trigger.Request()
                    # self.get_logger().info(f'Triggering')
                    future = self.gpio_client.call_async(trigger_request)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        self.get_logger().info(f'Image {n+1} captured')
                        time.sleep(0.150)
                    else:
                        self.get_logger().error('Service call failed')
            # Call laser service to turn off
            laser_request = SetBool.Request()
            laser_request.data = False  # Turn off the laser
            future_laser = self.laser_client.call_async(laser_request)
            rclpy.spin_until_future_complete(self, future_laser)
            if future_laser.result() is not None:
                self.get_logger().info('Laser turned off')

            rclpy.spin_until_future_complete(self, future_laser)
            response.success = True
            response.message = 'Images captured successfully'
            float_msg.data = 0.0  # Example value
            self.motor_angle_pub.publish(float_msg)
        return response

    def spatial_correl_process(self):
        """
        Function to perform spatial correlation
        """
        self.get_logger().info('Convert Images')
        Zscan.convert_images(self.left_images, self.right_images)

        self.get_logger().info('Construct 3D points')
        points_3d = Zscan.points3d(x_lim=(-300, 350), y_lim=(-400, 400), z_lim=(-800, 400), xy_step=15, z_step=2,
                                   visualize=False)
        self.get_logger().info('Spatial correlation')
        correl_points = Zscan.correlation_process(points_3d=points_3d, win_size=7, threshold=0.95)
        if correl_points[:,0].size < 0:
            self.get_logger().error('No points found')
            return
        self.get_logger().info('Second 3D points')
        xlim, ylim, zlim = [min(correl_points[:,0]), max(correl_points[:,0])], [min(correl_points[:,1]), max(correl_points[:,1])], [min(correl_points[:,2]), max(correl_points[:,2])]
        points_3d_2 = Zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, z_step=.5, xy_step=1, visualize=False)

        correl_points = Zscan.correlation_process(points_3d=points_3d_2, win_size=7, threshold=0.9)
        if correl_points[:,0].size < 0:
            self.get_logger().error('No points found')
            return
        self.get_logger().info('Publishing point cloud')
        if correl_points is not None:
            pcl_points = self.convert_to_pointcloud2(correl_points)
            self.pcl_publisher.publish(self.convert_to_pointcloud2(pcl_points))

            self.left_images, self.right_images = ndarray([]), ndarray([])
            rclpy.get_logger().info('Point cloud published')

    def convert_to_pointcloud2(self, points, frame_id="left_camera"):
        # Converte para mensagem PointCloud2
        header = Header()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pointcloud_data = b''.join([struct.pack('fff', *p) for p in points.tolist()])

        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points),
            data=pointcloud_data,
            is_dense=True
        )    

def main(args=None):
    rclpy.init(args=args)
    node = InverseTriangulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()