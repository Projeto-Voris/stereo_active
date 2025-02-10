#!/usr/bin/env python3
import os
import cv2
import numpy as np
import cupy as cp
import time
import struct
from InverseTriangulation import InverseTriangulation

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from std_msgs.msg import Header
import message_filters
from std_srvs.srv import Trigger, SetBool


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
        self.save_srv = self.create_service(Trigger, 'save_images', self.save_cb)

        self.count = 1
        self.perform_correl = False

        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.perform_correl:
            t0 = time.time()
            self.Zscan.convert_images(left_imgs=self.left_images, right_imgs=self.right_images, apply_clahe=True)
            self.get_logger().info('Correlation process started {:.2f}'.format(t0))
            self.spatial_correl_process()
            self.get_logger().info('Correlation process finished {:.2f}'.format(time.time()-t0))
            self.perform_correl = False
    
    def save_cb(self, request, response):
        """
        Service callback to view the point cloud
        """
        if request:
            self.get_logger().info('Viewing images')
            os.makedirs('left', exist_ok=True)
            os.makedirs('right', exist_ok=True)
            for n in range(self.Zscan.left_images.shape[2]):
                cv2.imwrite('left/L{:02d}.png'.format(n + 1), cp.asnumpy(self.Zscan.left_images[:,:,n]))
                cv2.imwrite('right/R{:02d}.png'.format(n + 1), cp.asnumpy(self.Zscan.right_images[:,:,n]))
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

        if self.count < self.num_images:
            self.get_logger().info('Images callback received - {}'.format(self.count))
        else:
            self.get_logger().info('Images received')
            return

        if left_image.encoding == 'bgr8' or right_image.encoding == 'bgr8':
            left_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
            right_images = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        else:
            left_images = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='mono8')
            right_images = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='mono8')
    
        self.left_images.append(left_images)
        self.right_images.append(right_images)
        self.count +=1


    def get_images_srv(self, request, response):
        """
        Service callback to get stereo images
        """
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        if request:
            self.count = 1
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
                for n in range(self.num_images+4):
                    # self.get_logger().info(f'Waiting for image {n+1}')
                    trigger_request = Trigger.Request()
                    # self.get_logger().info(f'Triggering')
                    future = self.gpio_client.call_async(trigger_request)
                    rclpy.spin_until_future_complete(self, future)
                    if future.result() is not None:
                        # self.get_logger().info(f'Image {n+1} captured')
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
            # self.srv_called = True
            float_msg.data = 0.0  # Example value
            self.motor_angle_pub.publish(float_msg)
                        # self.correlation_process();

            self.get_logger().info('Calling spatial correlation')
            self.perform_correl = True
        return response

    def spatial_correl_process(self):
        """
            Function to perform spatial correlation
        """
        # self.get_logger().info('First 3D points')

        # self.get_logger().info('Construct 3D points')
        points_3d = self.Zscan.points3d(x_lim=(-400, 400), y_lim=(-400, 400), z_lim=(-500, 500), xy_step=10, z_step=1,
                                    visualize=False) 
        self.get_logger().info('3D meshgrid pts: {} mi '.format(points_3d.shape[0] / 1e6))
        # correl_points = self.Zscan.correlation_process(points_3d=points_3d, win_size=21, threshold=0.6)
        uv_left = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='left')
        uv_right = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='right')
        spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=15, uv_left=uv_left, uv_right=uv_right)
        correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=0.75, std_thresh=15)
        correl_points = points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)]

        self.get_logger().info('First 3D points size: {}'.format(correl_points.size))
        
        del uv_left, uv_right, spatial_id, spatial_max, std_corr, points_3d
        if correl_points.size <= 0:
            self.get_logger().error('No points found')
            return
        # self.get_logger().info('Second 3D points')
        
        xlim = [min(correl_points[:,0]), max(correl_points[:,0])] 
        ylim = [min(correl_points[:,1]), max(correl_points[:,1])]
        zlim = [min(correl_points[:,2]), max(correl_points[:,2])]
        self.get_logger().info('Boundaries 3D points: {}'.format([xlim, ylim, zlim]))

        del correl_points

        points_3d = self.Zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, z_step=.1, xy_step=0.5, visualize=False)
        self.get_logger().info('3D meshgrid pts: {} mi '.format(points_3d.shape[0] / 1e6))

        uv_left = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='left')
        uv_right = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='right')
        spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=21, uv_left=uv_left, uv_right=uv_right)
        correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=0.8, std_thresh=15)
        correl_points = points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)]


        
        del uv_left, uv_right, spatial_id, spatial_max, std_corr

        if correl_points.size <= 0:
            self.get_logger().error('No points found')
            return

        self.get_logger().info('Publishing point cloud')
        # correl_points = self.Zscan.filter_points_by_depth(correl_points, depth_threshold=0.1, std_ratio=1)
        self.get_logger().info('Type of correl_points: {}'.format(type(correl_points)))

        measured_pts_camera = (self.Zscan.camera_params['left']['r'] @ (correl_points.T + self.Zscan.camera_params['left']['t'][:, None])).T

        if correl_points is not None:
            pcl_points = self.convert_to_pointcloud2(measured_pts_camera)
            self.pcl_publisher.publish(pcl_points)

            self.left_images, self.right_images = np.ndarray([]), np.ndarray([])
            self.get_logger().info('Point cloud published')


    def convert_to_pointcloud2(self, points, frame_id="left_camera_link"):
        # Converte para mensagem PointCloud2
        header = Header()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Corrige a escala dos pontos de metros para milímetros
        points = np.divide(points, 100.0)

        pointcloud_data = b''.join([struct.pack('fff', *p) for p in points])

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