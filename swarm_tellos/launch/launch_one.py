import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Launch a single drone.

d1_name = 'drone1'
d2_name = 'drone2'
d3_name = 'drone3'
d4_name = 'drone4'
d5_name = 'drone5'
d6_name = 'drone6'

d1_params = [{
        'drone_ip': '192.168.10.1',
        'command_port': 32783,
        'drone_port': 8889,
        'data_port': 8890,
        'video_port': 11111
    	}]
    	
d2_params = [{
        'drone_ip': '192.168.10.2',
        'command_port': 32784,
        'drone_port': 8889,
        'data_port': 8890,
        'video_port': 11111
    	}]
    	
d3_params = [{
        'drone_ip': '192.168.10.3',
        'command_port': 32785,
        'drone_port': 8889,
        'data_port': 8890,
        'video_port': 11111
    	}]
    	
d4_params = [{
        'drone_ip': '192.168.10.4',
        'command_port': 32786,
        'drone_port': 8889,
        'data_port': 8890,
        'video_port': 11111
    	}]
    	
d5_params = [{
        'drone_ip': '192.168.86.206',
        'command_port': 11005,
        'drone_port': 12005,
        'data_port': 13005,
        'video_port': 14005
    	}]

d6_params = [{
        'drone_ip': '192.168.86.206',
        'command_port': 11006,
        'drone_port': 12006,
        'data_port': 13006,
        'video_port': 14006
    	}]






def generate_launch_description():
    return LaunchDescription([
        # Rviz
        #ExecuteProcess(cmd=['rviz2', '-d', 'install/flock2/share/swarm_tellos/rviz/one.rviz'], output='screen'),

        # Drone Drivers
        Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver1', namespace=d1_name, parameters=d1_params),
        Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver2', namespace=d2_name, parameters=d2_params),
        Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver3', namespace=d3_name, parameters=d3_params),
        Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver4', namespace=d4_name, parameters=d4_params),
        #Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver5', namespace=d5_name, parameters=d5_params),
        #Node(package='tello_driver', executable='tello_driver_main', output='screen', name='driver6', namespace=d6_name, parameters=d6_params),
        

        #Drone Controllers
        Node(package='swarm_tellos', executable='drone_controller.py', output='screen',arguments= [d1_name]),
        Node(package='swarm_tellos', executable='drone_controller.py', output='screen',arguments= [d2_name]),
        Node(package='swarm_tellos', executable='drone_controller.py', output='screen',arguments= [d3_name]),
        Node(package='swarm_tellos', executable='drone_controller.py', output='screen',arguments= [d4_name]),


        # Joystick
        Node(package='swarm_tellos', executable='global_controller.py', output='screen',arguments= ["drone1"] ),
        #Node(package='joy', executable='joy_node', output='screen'),









    ])
