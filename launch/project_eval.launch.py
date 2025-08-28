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
                'left_image',
                default_value='left/image_raw',
                description='Left camera info topic'
            ),
            DeclareLaunchArgument(
                'right_image',
                default_value='right/image_raw',
                description='Right camera info topic'
            ),
            DeclareLaunchArgument(
                'n_images',
                default_value='30',
                description='Number of images to acquire'
            ),

            Node(
                package='stereo_active',
                executable='project_evaluation.py',
                name='project_eval_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'num_images': LaunchConfiguration('n_images')},
                ],
                remappings=[
                    ('left/image', LaunchConfiguration('left_image')),
                    ('right/image', LaunchConfiguration('right_image')),
                ]
            ),
            Node(
                package='stereo_active',
                executable='project_pattern.py',
                name='project_patarn_node',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[
                    {'image_path': '/home/jetson/Pictures/random_pattern'}
                ]
            )
        ])