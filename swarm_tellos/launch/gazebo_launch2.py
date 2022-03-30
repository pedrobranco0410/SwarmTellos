import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a single drone.


def generate_launch_description():
    return LaunchDescription([
        # Rviz
        #ExecuteProcess(cmd=['rviz2', '-d', 'install/flock2/share/flock2/launch/one.rviz'], output='screen'),


        # Joystick
        Node(package='swarm_tellos', executable='global_controller.py', output='screen',arguments= ["drone1", 'drone2', "CARALHO"] ),


        # Drone controller
        #Node(package='flock2', executable='drone_base', output='screen',
         #    name='drone_base', namespace='solo'),



    ])