from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument(
                'namespace',
                default_value='Active',
                description='Namespace'
            ),
            DeclareLaunchArgument(
                'point_cloud',
                default_value='SM3/pointcloud',
                description='Point cloud topic'
            ),
            DeclareLaunchArgument(
                'n_images',
                default_value='10',
                description='Number of images to acquire'
            ),
            DeclareLaunchArgument(
                'window_size',
                default_value='3',
                description='Number of images to acquire'
            ),
            DeclareLaunchArgument(
                'yaml_path',
                default_value=PathJoinSubstitution([FindPackageShare('stereo_active'), 'config','SM3.yaml']),
                description='Number of images to acquire'
            ),
            DeclareLaunchArgument(
                'camera_frame_id',
                default_value='Active/left_camera_link',
                description='Camera frame ID'
            ),
            Node(
                package='stereo_active',
                executable='new_inv_correlation_node.py',
                name='inverse_correlation_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'num_images': LaunchConfiguration('n_images'),
                     'yaml_path': LaunchConfiguration('yaml_path'),
                     'window_size': LaunchConfiguration('window_size'),
                     'camera_frame_id': LaunchConfiguration('camera_frame_id')}
                ],
                remappings=[
                    ('pointcloud', LaunchConfiguration('point_cloud')),
                ]
            )
        ])