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
                
            }]
        ),
        Node(
            package='perception',
            executable='QRcode_detection.py',
            name='QRcode_detection',
            output='screen',
            parameters=[{
                
            }]
        ),

        # Launch the Pepsi can detector Python node
        Node(
            package='perception',
            executable='pepsi_can_detection.py',
            name='pepsi_can_detection',
            output='screen',
            parameters=[{
                
            }]
        ),
        Node(
            package='perception',
            executable='lightsensordetection.py',
            name='lightsensordetection',
            output='screen',
            parameters=[{
               
            }]
        ),
        
    ])
