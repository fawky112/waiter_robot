from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'udp4', '--port', '8888'
            ],
            output='screen'
        ),

        Node(
            package='detection',
            executable='detect_node',
            name='detect_node',
            output='screen'
        ),

        Node(
            package='kiosk_robot',
            executable='bridge',
            name='kiosk_bridge',
            output='screen'
        ),
    ])
