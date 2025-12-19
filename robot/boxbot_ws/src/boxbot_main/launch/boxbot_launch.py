from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package="boxbot_main",
            executable="drive",
        ), 
        Node(
            package="boxbot_main", 
            executable="computer_vision", 
        ),
    ])