from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument(
                'namespace',
                default_value='SM3',
                description='Namespace'
            ),
            DeclareLaunchArgument(
                'left_camera_info',
                default_value='/SM2/left/camera_info',
                description='Left camera info topic'
            ),
            DeclareLaunchArgument(
                'right_camera_info',
                default_value='/SM2/right/camera_info',
                description='Right camera info topic'
            ),
            DeclareLaunchArgument(
                'left_image',
                default_value='/SM2/left/image_raw',
                description='Left camera info topic'
            ),
            DeclareLaunchArgument(
                'right_image',
                default_value='/SM2/right/image_raw',
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
            Node(
                package='stereo_active',
                executable='inv_correlation_node.py',
                name='inverse_correlation_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'num_images': LaunchConfiguration('n_images')}
                ],
                remappings=[
                    ('left/camera_info', LaunchConfiguration('left_camera_info')),
                    ('right/camera_info', LaunchConfiguration('right_camera_info')),
                    ('left/image', LaunchConfiguration('left_image')),
                    ('right/image', LaunchConfiguration('right_image')),
                    ('point_cloud', LaunchConfiguration('point_cloud')),
                    ('motor/angle', LaunchConfiguration('motor_topic'))
                ]
            )
        ])