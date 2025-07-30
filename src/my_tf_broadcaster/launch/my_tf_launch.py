from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Uruchomienie my_tf_broadcaster z argumentem odom_to_tf
        Node(
            package='my_tf_broadcaster',
            executable='odom_to_tf',
            name='my_tf_broadcaster'
        ),
        # Uruchomienie static_transform_publisher z odpowiednimi argumentami
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser']
        )
    ])