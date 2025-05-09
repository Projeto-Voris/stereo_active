#!/usr/bin/env python3
import os
import cv2
import numpy as np
import cupy as cp
import time
import struct
from InverseTriangulation import InverseTriangulation
from SpatialCorrelation import SpatialCorrelator
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

        # Parameters declaration
        self.declare_parameter('num_images', 15)
        self.declare_parameter('yaml_path', '~/ros2_ws/src/stereo_active/config/SM3.yaml')
        self.declare_parameter('tile', 15)
        self.declare_parameter('climp', 5.0)
        self.declare_parameter('threshold1', 0.85)
        self.declare_parameter('radius1', 40)
        self.declare_parameter('neighbors1', 10)
        self.declare_parameter('threshold2', 0.9)
        self.declare_parameter('radius2', 5)
        self.declare_parameter('neighbors2', 10)

        self.declare_parameter('save_correl', False)
        self.declare_parameter('save_points', False)

        self.num_images = self.get_parameter('num_images').get_parameter_value().integer_value
        self.yaml_file = self.get_parameter('yaml_path').get_parameter_value().string_value
        
        self.get_logger().info(f'Number of images to be captured: {self.num_images}')

        # Initialize the InverseTriangulation class
        # self.Zscan = InverseTriangulation(yaml_file=self.yaml_file)
        self.Zscan = SpatialCorrelator(yaml_file=self.yaml_file)
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
        self.srv = self.create_service(SetBool, 'process', self.get_images_srv, callback_group=self.callback_group_srv)
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
            self.Zscan.convert_images(left_imgs=self.left_images, right_imgs=self.right_images, apply_clahe=True, tile=tile, climp=climp, undist=True)
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

        if self.count <= self.num_images:
            self.get_logger().info('Images callback received - {}'.format(self.count))
        else:
            # self.get_logger().info('Images received')
            return

        if left_image.encoding == 'bgr8' or right_image.encoding == 'bgr8':
            left_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
            right_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        else:
            left_image = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='mono8')
            right_image = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='mono8')
    
        self.left_images.append(left_image)
        self.right_images.append(right_image)
        if self.service_requet:
            self.count +=1

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

    def spatial_correl_process(self):
        """
            Function to perform spatial correlation
        """
        thresh1 = self.get_parameter('threshold1').get_parameter_value().double_value
        win_size1 = self.get_parameter('window_size1').get_parameter_value().integer_value
        std_thresh1 = self.get_parameter('std_threshold1').get_parameter_value().integer_value
        thresh2 = self.get_parameter('threshold2').get_parameter_value().double_value
        win_size2 = self.get_parameter('window_size2').get_parameter_value().integer_value
        std_thresh2 = self.get_parameter('std_threshold2').get_parameter_value().integer_value
        save_correl = self.get_parameter('save_correl').get_parameter_value().bool_value
        # self.get_logger().info('First 3D points')
        add_distort = False

        # self.get_logger().info('Construct 3D points')
        points_3d = self.Zscan.points3d(x_lim=(-200, 600), y_lim=(-200, 500), z_lim=(-500, 500), xy_step=10, z_step=1,
                                    visualize=False) 
        self.get_logger().info('3D meshgrid pts: {} mi '.format(points_3d.shape[0] / 1e6))
        # correl_points = self.Zscan.correlation_process(points_3d=points_3d, win_size=21, threshold=0.6)
        uv_left = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='left', undist=add_distort)
        uv_right = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='right', undist=add_distort)
        spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=win_size1, uv_left=uv_left, uv_right=uv_right, save_points=save_correl, name_file='reshaped_1pts')
        correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=thresh1, std_thresh=std_thresh1)
        correl_points = points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)]

        self.get_logger().info('First 3D points size: {}'.format(correl_points.size))
        
        del uv_left, uv_right, spatial_id, spatial_max, std_corr, points_3d
            
        if correl_points.size <= 0:
            self.get_logger().error('No points found')
            return
        
        if correl_points is not None:
            pcl_points = self.convert_to_pointcloud2(correl_points)
            self.pcl_publisher.publish(pcl_points)

            # self.left_images, self.right_images = np.ndarray([]), np.ndarray([])
            self.get_logger().info('Point cloud published points: {}'.format(correl_points.shape[0]))



        self.get_logger().info('Second 3D points')
        
        xlim = [min(correl_points[:,0]), max(correl_points[:,0])] 
        ylim = [min(correl_points[:,1]), max(correl_points[:,1])]
        zlim = [min(correl_points[:,2]), max(correl_points[:,2])]

        if zlim[0] == zlim[1]:
            self.get_logger().warning('Z limits are equal')
            zlim[1] = zlim[0] + 1
            zlim[0] = zlim[0] - 1

        self.get_logger().info('Boundaries of first 3D points: {}'.format([xlim, ylim, zlim]))

        
        if abs(zlim[1] - zlim[0]) < 100:
            del correl_points
            points_3d = self.Zscan.points3d(x_lim=xlim, y_lim=ylim, z_lim=zlim, z_step=.1, xy_step=2, visualize=False)
            self.get_logger().info('3D meshgrid pts: {} mi '.format(points_3d.shape[0] / 1e6))


            uv_left = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='left', undist=add_distort)
            uv_right = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='right', undist=add_distort)
            spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=win_size2, uv_left=uv_left, uv_right=uv_right, save_points=save_correl, name_file='reshaped_2pts')
            correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=thresh2, std_thresh=std_thresh2)
            correl_points = points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)]

            
            del uv_left, uv_right, spatial_id, spatial_max, std_corr

        else:
            self.get_logger().info('Z limits are too far apart')


            split_pts = np.array_split(correl_points, 10, axis=0)
            # del correl_points
            x_lin_r = np.arange(xlim[0], xlim[1], 2)
            y_lin_r = np.arange(ylim[0], ylim[1], 2)
            
            x_lin_split = np.array_split(x_lin_r, 3)
            y_lin_split = np.array_split(y_lin_r, 3)

            scnd_pts = []
            for i, x_sp in enumerate(x_lin_split):
                for j, y_sp in enumerate(y_lin_split):
                    t0 = time.time()
                    # Filter correl_points based on x_sp and y_sp
                    mask = (correl_points[:, 0] >= x_sp[0]) & (correl_points[:, 0] <= x_sp[-1]) & \
                        (correl_points[:, 1] >= y_sp[0]) & (correl_points[:, 1] <= y_sp[-1])
                    filtered_points = correl_points[mask]

                    if filtered_points.size == 0:
                        self.get_logger().warning('No points found for {},{} split'.format(i,j))
                        continue

                    zlim_split = [min(filtered_points[:, 2]), max(filtered_points[:, 2])]
                    z_lin_r = np.arange(zlim_split[0], round(zlim_split[1],1), 1)

                    xlim = [min(x_sp), max(x_sp)] 
                    ylim = [min(y_sp), max(y_sp)]

                    if z_lin_r.size > 0:
                        zlim = [min(z_lin_r), max(z_lin_r)]
                    else:
                        self.get_logger().warning('z_lin_r is empty for {},{} split, skipping this segment'.format(i,j))
                        continue

                    self.get_logger().info('Boundaries - {},{} 3D points: {}'.format(i, j, [xlim, ylim, zlim]))

                    points_3d = self.Zscan.point3d_split(x_lin=x_sp, y_lin=y_sp, z_lin=z_lin_r, visualize=False)
                    self.get_logger().info('3D meshgrid pts: {} mi '.format(points_3d.shape[0] / 1e6))
                

                    uv_left = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='left', undist=add_distort)
                    uv_right = self.Zscan.transform_gcs2ccs(points_3d=points_3d, cam_name='right', undist=add_distort)
                    spatial_id, spatial_max, std_corr = self.Zscan.spatial_correl(window_size=win_size2, uv_left=uv_left, uv_right=uv_right, save_points=save_correl, name_file='reshaped_2pts_{}_{}'.format(i,j))
                    correl_mask = self.Zscan.correl_mask(std_correl=std_corr, correl_max=spatial_max, correl_thresh=thresh2, std_thresh=std_thresh2)
                    scnd_pts.append(points_3d[np.asarray(cp.asnumpy(spatial_id[correl_mask])).astype(np.int32)])

                    del uv_left, uv_right, spatial_id, spatial_max, std_corr
                    self.get_logger().info('Time for {},{} split: {:.2f} s'.format(i,j, time.time()-t0))

            correl_points = np.concatenate(scnd_pts, axis=0)


        if correl_points.size <= 0:
            self.get_logger().error('No points found')
            return

        # self.Zscan.save_points(points=correl_points, filename='correl_i{}_{}_{}_{}.txt'.format(self.num_images, thresh2, win_size2, std_thresh2))
        self.get_logger().info('Publishing point cloud')


        if correl_points is not None:
            pcl_points = self.convert_to_pointcloud2(correl_points)
            self.pcl_publisher.publish(pcl_points)

            self.get_logger().info('Point cloud published refined points: {}'.format(correl_points.shape[0]))
        else:
            self.get_logger().error('No points found')

    def spatial_3d_correl_process(self):
        """
            Function to perform spatial correlation
        """
        thresh1 = self.get_parameter('threshold1').get_parameter_value().double_value
        radius1 = self.get_parameter('radius1').get_parameter_value().integer_value
        neighbors1 = self.get_parameter('neighbors1').get_parameter_value().integer_value
        thresh2 = self.get_parameter('threshold2').get_parameter_value().double_value
        radius2 = self.get_parameter('radius2').get_parameter_value().integer_value
        neighbors2 = self.get_parameter('neighbors2').get_parameter_value().integer_value
        save_correl = self.get_parameter('save_correl').get_parameter_value().bool_value
        save_points = self.get_parameter('save_points').get_parameter_value().bool_value    
        # self.get_logger().info('First 3D points')
        
        # self.get_logger().info('3D meshgrid pts: {} mi '.format(self.Zscan.grid.shape[0] / 1e6))
        self.Zscan.points3d(x_lim=(-0,400), y_lim=(-100,300), z_lim=(-400,400), xy_step=20, z_step=1)
        # 
        xyz, corr, _, _ = self.Zscan.run_batch(r_xy=1, stride=2)
        xyz = cp.asnumpy(xyz[corr > thresh1])
        corr = cp.asnumpy(corr[corr > thresh1])
        filtered_xyz, filtered_corr = self.Zscan.filter_sparse_points(xyz=xyz, corr=corr, min_neighbors=neighbors1, radius=radius1)


        self.get_logger().info('First 3D points size: {}'.format(filtered_xyz.shape[0]))
                    
        if filtered_xyz.size <= 0:
            self.get_logger().error('No points found')
            return
        
        if filtered_xyz is not None:
            pcl_points = self.convert_to_pointcloud2(filtered_xyz)
            self.pcl_publisher.publish(pcl_points)

            # self.left_images, self.right_images = np.ndarray([]), np.ndarray([])
            self.get_logger().info('Point cloud published points: {}'.format(filtered_xyz.shape[0]))

        if save_correl:
            np.savetxt('correl_1nd_th{:.1f}_{}.txt'.format(thresh1, time.strftime("%Y%m%d_%H%M")), corr.flatten(), fmt='%.6f')


        self.get_logger().info('Second 3D points')
        
        xlim = [min(filtered_xyz[:,0]), max(filtered_xyz[:,0])] 
        ylim = [min(filtered_xyz[:,1]), max(filtered_xyz[:,1])]
        zlim = [min(filtered_xyz[:,2]), max(filtered_xyz[:,2])]

        if zlim[0] == zlim[1]:
            self.get_logger().warning('Z limits are equal')
            zlim[1] = zlim[0] + 1
            zlim[0] = zlim[0] - 1
            self.Zscan.points3d(xlim, ylim, zlim, xy_step=1, z_step=1)
            xyz, corr, _, _ = self.Zscan.run_batch(r_xy=.5, stride=2)
            xyz = cp.asnumpy(xyz[corr > thresh2])
            corr = cp.asnumpy(corr[corr > thresh2])
            filtered_xyz, filtered_corr = self.Zscan.filter_sparse_points(xyz=xyz, corr=corr, min_neighbors=neighbors2, radius=radius2)


        self.get_logger().info('Boundaries of first 3D points: {}'.format([xlim, ylim, zlim]))

        
        
        del filtered_xyz

        self.Zscan.points3d(xlim, ylim, zlim, xy_step=1, z_step=1)
        xyz, corr, _, _ = self.Zscan.run_batch(r_xy=.5, stride=2)
        xyz = cp.asnumpy(xyz[corr > thresh2])
        corr = cp.asnumpy(corr[corr > thresh2])
        filtered_xyz, filtered_corr = self.Zscan.filter_sparse_points(xyz=xyz, corr=corr, min_neighbors=neighbors2, radius=radius2)

        if filtered_xyz.size <= 0:
            self.get_logger().error('No points found')
            return

        self.get_logger().info('Publishing point cloud')

        if save_correl:
            np.savetxt('correl_2nd_th{:.1f}_{}.txt'.format(thresh2, time.strftime("%Y%m%d_%H%M")), corr.flatten(), fmt='%.6f')

        if save_points:
            np.savetxt('points_{}.txt'.format(time.strftime("%Y%m%d_%H%M")), filtered_xyz, fmt='%.6f')
            
        if filtered_xyz is not None:
            pcl_points = self.convert_to_pointcloud2(filtered_xyz)
            self.pcl_publisher.publish(pcl_points)

            self.get_logger().info('Point cloud published refined points: {}'.format(filtered_xyz.shape[0]))
        else:
            self.get_logger().error('No points found')

    def convert_to_pointcloud2(self, points, frame_id="SM3/left_camera_link"):

        points = (np.eye(3) @ (points.T + self.Zscan.camera_params['left']['t'][:, None])).T

        # Converte para mensagem PointCloud2
        header = Header()
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