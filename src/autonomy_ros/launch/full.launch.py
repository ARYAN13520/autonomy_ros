from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='autonomy_ros',
            executable='mavlink',
            name='mavlink_node',
            output='screen'
        ),

    ])

