from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument(
                'namespace',
                default_value='SM3',
                description='Namespace'
            ),
            DeclareLaunchArgument(
                'left_image',
                default_value='/SM3/left/image_raw',
                description='Left camera info topic'
            ),
            DeclareLaunchArgument(
                'right_image',
                default_value='/SM3/right/image_raw',
                description='Right camera info topic'
            ),
            DeclareLaunchArgument(
                'point_cloud',
                default_value='point_cloud',
                description='Point cloud topic'
            ),
            DeclareLaunchArgument(
                'motor_topic',
                default_value='motor/angle',
                description='Point cloud topic'
            ),
            DeclareLaunchArgument(
                'n_images',
                default_value='10',
                description='Number of images to acquire'
            ),
            DeclareLaunchArgument(
                'yaml_path',
                default_value=PathJoinSubstitution([FindPackageShare('stereo_active'), 'config','SM3.yaml']),
                description='Number of images to acquire'
            ),

            Node(
                package='stereo_active',
                executable='inv_correlation_node.py',
                name='inverse_correlation_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'num_images': LaunchConfiguration('n_images'),
                     'yaml_path': LaunchConfiguration('yaml_path')}
                ],
                remappings=[
                    ('left/image', LaunchConfiguration('left_image')),
                    ('right/image', LaunchConfiguration('right_image')),
                    ('point_cloud', LaunchConfiguration('point_cloud')),
                    ('motor/angle', LaunchConfiguration('motor_topic'))
                ]
            )
        ])