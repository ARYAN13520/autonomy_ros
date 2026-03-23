from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='autonomy_ros',
            executable='mavlink',
            name='mavlink_node'
        ),

        Node(
            package='autonomy_ros',
            executable='controller',
            name='controller_node'
        ),
        Node(
            package='autonomy_ros',
            executable='flight',
            name='flight_service'
        ),

    ])
