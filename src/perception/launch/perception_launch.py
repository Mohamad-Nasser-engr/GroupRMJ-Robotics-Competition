from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='nodes/QRcode_detection.py',  # Adjusted path for the Python node
            name='QRcode_detection',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }]
        ),
        Node(
            package='perception',
            executable='nodes/pepsi_can_detection.py',  # Adjusted path for the Python node
            name='pepsi_can_detection',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }]
        ),
        Node(
            package='perception',
            executable='nodes/lightsensordetection.py',  # Adjusted path for the Python node
            name='lightsensordetection',
            output='screen',
            parameters=[{
                # Add parameters here if needed
            }]
        ),
    ])


