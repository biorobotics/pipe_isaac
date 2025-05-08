from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {
                    'deadzone': 0.01
                }
            ]
        )
    
    tf_publisher = Node(
        package='tf2_ros',
        # namespace = '',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "scan", "world"]
    )

    robot_driver = Node(
            package='pipe_control',
            executable='joy_sim_driver',
            name='joy_sim_driver_node',
            output='screen'
        )

    return LaunchDescription([
        joy_node,
        tf_publisher,
        robot_driver
    ])