from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='initNode',
            name='init_node',
            output='screen',
            parameters=[{
                # Add any parameters you want to set for the node here
            }]
        ),
        Node(
            package='perception',
            executable='QRcode_detection.py',
            name='QRcode_detection',
            output='screen',
            parameters=[{
                # Add any parameters specific to QR code detection here
            }]
        ),

        # Launch the Pepsi can detector Python node
        Node(
            package='perception',
            executable='pepsi_can_detection.py',
            name='pepsi_can_detection',
            output='screen',
            parameters=[{
                # Add any parameters specific to Pepsi can detection here
            }]
        ),
    ])
