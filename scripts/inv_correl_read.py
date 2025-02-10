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
from sensor_msgs.msg import PointCloud2, PointField

from std_msgs.msg import Header
import message_filters
from std_srvs.srv import Trigger


class CorrelNode(Node):
    def __init__(self):
        super().__init__('correl_images_folder')
        self.get_logger().info('Correl Node from stored images started.')

        self.declare_parameter('num_images', 10)
        self.declare_parameter('yaml_path', '/home/jetson/ros2_ws/src/stereo_active/config/SM3.yaml')
        self.declare_parameter('imgs_path', '/home/jetson/Pictures/20250205')

        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.yaml_file = self.get_parameter('yaml_path').get_parameter_value().string_value
        self.imgs_path = self.get_parameter('imgs_path').get_parameter_value().string_value

        self.get_logger().info(f'Number of images to be used: {self.num_images}')

        self.Zscan = InverseTriangulation(yaml_file=self.yaml_file)
        # self.get_logger().info('{}'.format(self.Zscan.camera_params['left']['kc']))
        # self.get_logger().info('{}'.format(np.asarray(self.Zscan.camera_params['right']['kc'])))

        self.pcl_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        self.img_srv = self.create_service(Trigger, 'read_images', self.get_images_srv)
        self.img_view_srv = self.create_service(Trigger, 'view_images', self.view_images_srv)
        self.inv_srv = self.create_service(Trigger, 'inv_correl', self.correl_cb)

        self.perform_correl = False
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def timer_callback(self):
        if self.perform_correl:
            t0 = time.time()
            self.get_logger().info('Correlation process started {:.2f}'.format(t0))
            self.spatial_correl_process()
            self.get_logger().info('Correlation process finished {:.2f}'.format(time.time()-t0))
            self.perform_correl = False

    def view_images_srv(self, request, response):
        """
        Service callback to view stereo images
        """
        if request:
            left_imgs = cp.asnumpy(self.Zscan.left_images)
            right_imgs = cp.asnumpy(self.Zscan.right_images)

            if left_imgs is not None and right_imgs is not None:
                for i in range(left_imgs.shape[2]):
                    concatenated_img = np.concatenate((left_imgs[:, :, i], right_imgs[:, :, i]), axis=1)
                    cv2.namedWindow('Stereo', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('Stereo', int(concatenated_img.shape[1] / 4), int(concatenated_img.shape[0] / 4))
                    cv2.imshow('Stereo', concatenated_img)
                    cv2.waitKey(0)  # Wait for a key press to show next image
                cv2.destroyAllWindows()
                response.success = True
                response.message = 'Images displayed'
            else:
                response.success = False
                response.message = 'No images to display'
        return response
        
    def get_images_srv(self, request, response):
        """
        Service callback to get stereo images
        """
        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.img_path = self.get_parameter('imgs_path').get_parameter_value().string_value
        if request:
            left_imgs = self.Zscan.read_images(path=os.path.join(self.img_path, 'left'), n_imgs=self.num_images,
                                                images_list=sorted(os.listdir(os.path.join(self.img_path, 'left'))))
            right_imgs = self.Zscan.read_images(path=os.path.join(self.img_path, 'right'), n_imgs=self.num_images,
                                                images_list=sorted(os.listdir(os.path.join(self.img_path, 'right'))))
            # self.get_logger().info('Loaded left imgs: {}'.format(left_imgs.shape))
            # self.get_logger().info('Loaded right imgs: {}'.format(right_imgs.shape))
            if self.Zscan.convert_images(left_imgs, right_imgs, apply_clahe=True):
                response.success = True
                response.message = 'Images loaded'
            else:
                response.success = False
                response.message = 'Error loading images'
        return response

    def correl_cb(self, request, response):
        if request:
            self.perform_correl = True
            response.success = True
            response.message = 'Correlation process started'
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
        spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=11, uv_left=uv_left, uv_right=uv_right)
        correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=0.75, std_thresh=20)
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
        spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=17, uv_left=uv_left, uv_right=uv_right)
        correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=0.8, std_thresh=20)
        correl_points = points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)]


       
        del uv_left, uv_right, spatial_id, spatial_max, std_corr

        if correl_points.size <= 0:
            self.get_logger().error('No points found')
            return

        self.get_logger().info('Publishing point cloud')
        correl_points = self.Zscan.filter_points_by_depth(correl_points, depth_threshold=0.1, std_ratio=1)
        self.get_logger().info('Type of correl_points: {}'.format(type(correl_points)))

        if correl_points is not None:
            pcl_points = self.convert_to_pointcloud2(correl_points)
            self.pcl_publisher.publish(pcl_points)

            self.left_images, self.right_images = np.ndarray([]), np.ndarray([])
            self.get_logger().info('Point cloud published')

    def convert_to_pointcloud2(self, points, frame_id="left_camera"):
        # Transform from camera from object to camera's coordinate system

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
    node = CorrelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()