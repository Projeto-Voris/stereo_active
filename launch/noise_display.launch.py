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
            'service_topic',
            default_value='pattern_change',
            description='Namespace for node'
        ),
        DeclareLaunchArgument(
            'monitor_name',
            default_value='Monitor_0',
            description='Monitor output port'
        ),
        DeclareLaunchArgument(
            'noise_freq',
            default_value='20.0',
            description='Random image noise frequence'
        ),
        DeclareLaunchArgument(
            'noise_persitence',
            default_value='0.50',
            description='Persitence of image noise frequence'
        ),
        DeclareLaunchArgument(
            'noise_lacunarity',
            default_value='10.0',
            description='Lacunarity of random image noise'
        ),
        DeclareLaunchArgument(
            'noise_octave',
            default_value='7.0',
            description='Octave count of random image noise'
        ),
        Node(
            package='stereo_active',  # Replace with your package name
            executable='noise_image',  # Replace with your executable name
            namespace=LaunchConfiguration('namespace'),
            name='noise_image',
            output='screen',
            parameters=[
                {'frequency': LaunchConfiguration('noise_freq')},
                {'persistence': LaunchConfiguration('noise_persitence')},
                {'lacunarity': LaunchConfiguration('noise_lacunarity')},
                {'octave': LaunchConfiguration('noise_octave')},
                {'monitor_name': LaunchConfiguration('monitor_name')}
            ],
            remappings=[
                ('pattern_change', LaunchConfiguration('service_topic'))
            ]
        ),
    ])
