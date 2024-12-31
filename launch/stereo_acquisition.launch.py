from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='SM3',
            description='Namespace for node'
        ),
        DeclareLaunchArgument(
            'left_topic',
            default_value='/SM2/left/image_raw',
            description='Monitor output port'
        ),
        DeclareLaunchArgument(
            'right_topic',
            default_value='/SM2/right/image_raw',
            description='Random image noise frequence'
        ),
        DeclareLaunchArgument(  
            'service_topic',
            default_value='pattern_change',
            description='Persitence of image noise frequence'
        ),
        DeclareLaunchArgument(
            'n_images',
            default_value='10',
            description='Number of images to acquire'
        ),

        Node(
            package='stereo_active',  # Replace with your package name
            executable='stereo_acq',  # Replace with your executable name
            namespace=LaunchConfiguration('namespace'),
            name='stereo_acquisition',
            output='screen',
            parameters=[
                {'buffer_size': LaunchConfiguration('n_images')}
            ],
            remappings=[
                ('left_image', LaunchConfiguration('left_topic')),
                ('right_image', LaunchConfiguration('right_topic')),
                ('pattern_change', LaunchConfiguration('service_topic'))
            ]
        ),
    ])
