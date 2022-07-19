from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lgs_state_machine',
            executable='state_machine',
            name='state_machine_node',
            output='screen',
            parameters=[
                {'pipe_length': 158}
            ]
        )
    ])