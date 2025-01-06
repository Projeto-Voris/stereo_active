#!/usr/bin/env python3
import os
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import Header

from scripts.InverseTriangulation import InverseTriangulation

class InverseTriangulationNode(Node):
    def __init__(self):
        super().__init__('inverse_triangulation_node')
        self.Zscan = InverseTriangulation()
        self.get_logger().info('InverseTriangulationNode has been started.')
        self.declare_parameter('image_path', '/home/voris/Pictures/SM3/temp')

        self.image_path = self.get_parameter('images_path').get_parameter_value().string_value

        if not os.path.exists(self.image_path) or not os.listdir(self.image_path):
            self.get_logger().error('No images found in the specified path')
            return

        self.create_subscription(CameraInfo, 'left/camera_info', self.camera_info_left_cb, 1)
        self.create_subscription(CameraInfo, 'right/camera_info', self.camera_info_right_cb, 1)

        self.pcl_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.srv = self.create_service(Trigger, 'process_pointcloud', self.process_pointcloud_cb)


    
    def process_pointcloud_cb(self, request, response):
        """
        Service callback function to read images from the specified path
        """
        if (request.trigger):
            self.Zscan.read_images(os.path.join(self.image_path, 'right'), 
                                                                sorted(os.listdir(os.path.join(self.image_path, 'right'))),
                                                                CLAHE=True)
            self.Zscan.read_images(os.path.join(self.image_path, 'left'), 
                                                                sorted(os.listdir(os.path.join(self.image_path, 'left'))),
                                                                CLAHE=True)
            self.spatial_correl_process()
            response.success = True
        return response

    def camera_info_left_cb(self, msg):
        """
        Callback function for the left camera info subscriber
        """
        # Extract intrinsic parameters
        self.Zscan.camera_params['left']['kk'] = np.array(msg.k).reshape(3, 3)
        self.Zscan.camera_params['left']['kc'] = np.array(msg.d).reshape(5, 1)

        # Transform projection to rotation and translation vectors
        p_l = np.array(msg.p).reshape(3, 4)
        self.Zscan.camera_params['left']['r'] = np.array(msg.r).reshape(3,3)
        self.Zscan.camera_params['left']['t'] = np.array(msg.p).reshape(4,3)[:, 3]

    def camera_info_right_cb(self, msg):
        """
        Callback function for the right camera info subscriber
        """

        # Extract intrinsic parameters
        self.Zscan.camera_params['right']['kk'] = np.array(msg.k).reshape(3, 3)
        self.Zscan.camera_params['right']['kc'] = np.array(msg.d).reshape(5, 1)
        
        # Transform projection to rotation and translation vectors
        self.Zscan.camera_params['left']['r'] = np.array(msg.r).reshape(3, 3)
        self.Zscan.camera_params['right']['t'] = np.array(msg.p).reshape(4, 3)[:, 3]

        self.Zscan.camera_params['stereo']['R'] = np.array(msg.p).reshape(4, 3)[:3, :3]
        self.Zscan.camera_params['stereo']['T'] = np.array(msg.p).reshape(4, 3)[:, 3]

    def spatial_correl_process(self):
        """
        Function to perform spatial correlation
        """
        points_3d = Zscan.points3d(x_lim=(-300, 350), y_lim=(-400, 400), z_lim=(-800, 400), xy_step=15, z_step=2,
                                   visualize=False)

        correl_points = Zscan.correlation_process(points_3d=points_3d, win_size=7, threshold=0.95)

        xlim, ylim, zlim = [min(correl_points[:,0]), max(correl_points[:,0])], [min(correl_points[:,1]), max(correl_points[:,1])], [min(correl_points[:,2]), max(correl_points[:,2])]
        points_3d_2 = Zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, z_step=.5, xy_step=1, visualize=False)

        correl_points = Zscan.correlation_process(points_3d=points_3d_2, win_size=7, threshold=0.9)

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