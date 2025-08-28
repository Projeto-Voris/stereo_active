#!/usr/bin/env python3
import os
import cv2
import numpy as np
import time
import struct
import torch

from SpatialCorrelation_torch import PyTorchStereoCorrel as SpatialCorrelator

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from std_msgs.msg import Header
import message_filters


class InverseTriangulationNode(Node):
    def __init__(self):
        super().__init__('inverse_triangulation_node')
        self.get_logger().info('InverseTriangulationNode has been started.')

        # Parameters declaration
        self.declare_parameter('num_images', 5)
        self.declare_parameter('yaml_path', '~/ros2_ws/src/stereo_active/config/SM3.yaml')
        self.declare_parameter('tile', 1)
        self.declare_parameter('climp', 2.0)
        self.declare_parameter('window_size', 3)
        self.declare_parameter('stride', 1)
        self.declare_parameter('threshold', 0.75)
        self.declare_parameter('std_thresh', 10)
        self.declare_parameter('radius', 10)
        self.declare_parameter('neighbours', 10)


        self.declare_parameter('save_filename', "correlation_points")
        self.declare_parameter('debug_save_points', True)

        self.declare_parameter('camera_frame_id', 'SM3/left_camera_link')
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.yaml_file = self.get_parameter('yaml_path').get_parameter_value().string_value
        
        self.get_logger().info(f'Number of images to be captured: {self.num_images}')

        # Initialize the InverseTriangulation class
        # self.zscan = InverseTriangulation(yaml_file=self.yaml_file)
        self.zscan = SpatialCorrelator(yaml_file=self.yaml_file)
        self.bridge = CvBridge()

        self.left_images = []
        self.right_images = []

        # Initialize the subscribers
        self.left_image_sub = message_filters.Subscriber(self, Image, 'left/image')
        self.right_image_sub = message_filters.Subscriber(self, Image, 'right/image')
        # Initialize the publisher
        self.motor_angle_pub = self.create_publisher(Float32, 'motor/angle', 10)
        self.pcl_publisher = self.create_publisher(PointCloud2, 'pointcloud', 10)


        # Synchronize the stereo images
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub],
                                                    queue_size=10, slop=0.05)
        self.ts.registerCallback(self.stereo_images_callback)


        # Create mutually exclusive callback groups
        self.callback_group_srv = MutuallyExclusiveCallbackGroup()
        self.callback_group_trigger_client = MutuallyExclusiveCallbackGroup()
        self.callback_group_laser_client = MutuallyExclusiveCallbackGroup()

        # Create the service from node
        self.srv = self.create_service(SetBool, 'correlation_process', self.get_images_srv, callback_group=self.callback_group_srv)
        self.gpio_client = self.create_client(Trigger, 'trigger', callback_group=self.callback_group_trigger_client)
        self.laser_client = self.create_client(SetBool, 'laser', callback_group=self.callback_group_laser_client)
        self.save_srv = self.create_service(Trigger, 'save', self.save_cb)

        self.count = 1
        self.perform_correl = False
        self.service_requet = False

        # Timer to perform the correlation process
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        tile = self.get_parameter('tile').get_parameter_value().integer_value
        climp = self.get_parameter('climp').get_parameter_value().double_value

        if self.perform_correl and self.num_images <= self.count:
            t0 = time.time()
            self.zscan.convert_images(left_imgs_cpu=self.left_images, right_imgs_cpu=self.right_images, apply_clahe=True, undist=True, tile=tile, climp=climp)
            self.get_logger().info('Images converted: {:.2f} s'.format(time.time()-t0))
            self.spatial_3d_correl_process()
            self.get_logger().info('Correlation process finished: {:.2f} s'.format(time.time()-t0))
            self.perform_correl = False
            # self.left_images, self.right_images = [], []
            # self.count = 1
    
    def save_cb(self, request, response):
        """
        Service callback to view the point cloud
        """
        if request:
            self.get_logger().info('Saving images')
            os.makedirs('left', exist_ok=True)
            os.makedirs('right', exist_ok=True)
            n = 1
            for left, right in zip(self.left_images, self.right_images):
                cv2.imwrite('left/L{:02d}.png'.format(n), left)
                cv2.imwrite('right/R{:02d}.png'.format(n), right)
                n += 1
            if len(os.listdir('./left/')) == self.num_images:
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
        if self.service_requet and self.count <= self.num_images:
            self.get_logger().info('Captured stereo images: {}/{}'.format(self.count, self.num_images))
            self.count +=1
        else:
            return


        if left_image.encoding == 'bgr8' or right_image.encoding == 'bgr8':
            left_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
            right_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        else:
            left_image = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='mono8')
            right_image = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='mono8')
    
        self.left_images.append(left_image)
        self.right_images.append(right_image)

    def get_images_srv(self, request, response):
        """
        Service callback to get stereo images
        """
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.service_requet = True

        self.count = 1
        self.left_images, self.right_images = [], []
        float_msg = Float32()
        float_msg.data = 500.0  # Example value
        self.motor_angle_pub.publish(float_msg)

        # Call laser service
        laser_request = SetBool.Request()
        laser_request.data = True  # Turn on the laser
        future_laser = self.laser_client.call_async(laser_request)
        rclpy.spin_until_future_complete(self, future_laser)

        # If laser service was successful, trigger the camera
        if future_laser.result() is not None:
            self.get_logger().info('Laser turned on')
            for n in range(self.num_images+4):
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
        rclpy.spin_until_future_complete(self, future_laser)
        if future_laser.result() is not None:
            self.get_logger().info('Laser turned off')

        rclpy.spin_until_future_complete(self, future_laser)
        response.success = True
        response.message = 'Images captured successfully'
        float_msg.data = 0.0  # Example value
        self.motor_angle_pub.publish(float_msg)

        self.perform_correl = request.data
        return response


    def spatial_3d_correl_process(self):
        """
            Function to perform spatial correlation
        """


        # Get filter points parameters
        std_tresh = self.get_parameter('std_thresh').value
        rad_tresh = self.get_parameter('threshold').value
        win_size = self.get_parameter('window_size').get_parameter_value().integer_value
        stride = self.get_parameter('stride').get_parameter_value().integer_value

        radius = self.get_parameter('radius').value
        min_neighbours = self.get_parameter('neighbours').value
        save_points = self.get_parameter('debug_save_points').value
        filename = self.get_parameter('save_filename').get_parameter_value().string_value


        GRID_LIMITS = {'x': (-100, 500), 'y': (-100, 400), 'z': (-300, 800)}
        GRID_STEPS_1 = {'xy': 2.0, 'z': 4} # first steps of 3d patch
        GRID_STEPS_2 = {'xy': 2.0, 'z': 0.3} # second steps of 3d patch
        GRID_STEPS_3= {'xy': 1.0, 'z': 0.1} # second steps of 3d patch

        self.zscan.points3d(x_lim=GRID_LIMITS['x'], y_lim=GRID_LIMITS['y'], z_lim=GRID_LIMITS['z'],
                            xy_step=GRID_STEPS_1['xy'], z_step=GRID_STEPS_1['z'])
                        
        xyz_gpu, corr_gpu, _ = self.zscan.process_segmented_z(Kx=win_size, Ky=win_size, stride=stride, Nz_block_voxels=20, method='correl')

        # filter points based on difference value in radians
        filter_mask = corr_gpu < rad_tresh
        xyz_filtered_gpu = xyz_gpu[filter_mask]
        corr_filtered_gpu = corr_gpu[filter_mask]
        xyz_filtered_gpu, corr_filtered_gpu = self.zscan.mask_points(xyz_filtered_gpu, corr_filtered_gpu, bounds=std_tresh+10, method='correl')

        # clean points based on neighbours
        final_xyz_gpu, _ = self.zscan.filter_sparse_points( xyz_gpu=xyz_filtered_gpu, corr_gpu=corr_filtered_gpu,min_neighbours=min_neighbours, radius=radius)

        if final_xyz_gpu.numel() == 0:
            self.get_logger().warning("No points found")
            return
        
        # Find first 3D bounds to refined process               
        xlim = torch.min(final_xyz_gpu[:, 0]), torch.max(final_xyz_gpu[:, 0])
        ylim = torch.min(final_xyz_gpu[:, 1]), torch.max(final_xyz_gpu[:, 1])
        zlim = torch.min(final_xyz_gpu[:, 2]), torch.max(final_xyz_gpu[:, 2])

        if zlim[0] == zlim[1]:
            self.get_logger().info("Z are same")
            return

        # Construct second 3d points
        self.zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, 
                            xy_step=GRID_STEPS_2['xy'], z_step=GRID_STEPS_2['z'])
                        
        xyz_gpu, corr_gpu, _ = self.zscan.process_segmented_z(Kx=win_size, Ky=win_size, stride=stride, Nz_block_voxels=5, method='correl')

        
        filter_mask = corr_gpu > rad_tresh
        xyz_filtered_gpu = xyz_gpu[filter_mask]
        corr_filtered_gpu = corr_gpu[filter_mask]
        xyz_filtered_gpu, corr_filtered_gpu = self.zscan.mask_points(xyz_filtered_gpu, corr_filtered_gpu, bounds=std_tresh, method='correl')
        final_xyz_gpu, _ = self.zscan.filter_sparse_points(xyz_gpu=xyz_filtered_gpu, corr_gpu=corr_filtered_gpu, min_neighbours=min_neighbours+5, radius=radius-5)

        # if final_xyz_gpu.numel() == 0:
        #     self.get_logger().warning("No points found")
        #     return
        
        # # Find first 3D bounds to refined process               
        # xlim = torch.min(final_xyz_gpu[:, 0]), torch.max(final_xyz_gpu[:, 0])
        # ylim = torch.min(final_xyz_gpu[:, 1]), torch.max(final_xyz_gpu[:, 1])
        # zlim = torch.min(final_xyz_gpu[:, 2]), torch.max(final_xyz_gpu[:, 2])

        # if zlim[0] == zlim[1]:
        #     self.get_logger().info("Z are same")
        #     zlim[0] = zlim[0] - 5
        #     zlim[1] = zlim[1] + 5

        # # Construct second 3d points
        # self.zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, 
        #                     xy_step=GRID_STEPS_3['xy'], z_step=GRID_STEPS_3['z'])
                        
        # xyz_gpu, corr_gpu, _ = self.zscan.process_segmented_z(Kx=win_size, Ky=win_size, stride=stride, Nz_block_voxels=5, method='correl')

        
        # filter_mask = corr_gpu > rad_tresh
        # xyz_filtered_gpu = xyz_gpu[filter_mask]
        # corr_filtered_gpu = corr_gpu[filter_mask]
        # xyz_filtered_gpu, corr_filtered_gpu = self.zscan.mask_points(xyz_filtered_gpu, corr_filtered_gpu, bounds=std_tresh, method='correl')
        # final_xyz_gpu, _ = self.zscan.filter_sparse_points(xyz_gpu=xyz_filtered_gpu, corr_gpu=corr_filtered_gpu, min_neighbours=min_neighbours+10, radius=radius-5)

        pcl_points = self.convert_to_pointcloud2(xyz_filtered_gpu.cpu().numpy())
        self.pcl_publisher.publish(pcl_points)


        if save_points:
            np.savetxt('{}_{}.txt'.format(time.strftime("%Y%m%d"), filename), xyz_filtered_gpu.cpu().numpy(), fmt='%.6f')
            



    def convert_to_pointcloud2(self, points):
        frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        t_left = self.zscan.camera_params['left']['t'].cpu().numpy().T[0]
        r_left = self.zscan.camera_params['left']['r'].cpu().numpy()
        points = (r_left @ points.T).T + t_left

        # Converte para mensagem PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Corrige a escala dos pontos de metros para milímetros
        points = np.divide(points, 1000.0)

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