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
                'images_path',
                default_value='/home/voris/Pictures/SM3/temp',
                description='Path to the images'
            ),
            DeclareLaunchArgument(
                'point_cloud',
                default_value='point_cloud',
                description='Point cloud topic'
            ),
            Node(
                package='stereo_active',
                executable='inv_correlation.py',
                name='inverse_triangulation_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'images_path': LaunchConfiguration('images_path')}
                ],
                remappings=[
                    ('left/camera_info', LaunchConfiguration('left_camera_info')),
                    ('right/camera_info', LaunchConfiguration('right_camera_info')),
                    ('point_cloud', LaunchConfiguration('point_cloud'))
                ]
            )
        ])