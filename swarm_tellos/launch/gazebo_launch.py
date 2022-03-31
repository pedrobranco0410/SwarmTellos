"""Simulate one or more Tello drones in Gazebo, using ArUco markers and fiducial_vlam for localization"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # 1 or more drones:
    drones = ['drone1', 'drone2', 'drone3', 'drone4']#, 'drone5', 'drone6']

    # Starting locations:
    starting_locations = [
        ['0.0', '0.0', '1', '0'],
        ['0.0', '0.3', '1', '0'],
        ['0.0', '0.6', '1', '0'],
        ['0.0', '0.9', '1', '0'],
        ['0.3', '0.0', '1', '0'],
        ['0.3', '0.3', '1', '0'],
        ['0.3', '0.6', '1', '0'],
        ['0.3', '0.9', '1', '0'],
    ]

    tello_description_path = get_package_share_directory('swarm_tellos')

    world_path = os.path.join(get_package_share_directory('swarm_tellos'), 'worlds', 'simple.world')

    # Global entities
    entities = [
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',      # Publish /clock
            '-s', 'libgazebo_ros_factory.so',   # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

       #ExecuteProcess(cmd=['rviz2', '-d', 'install/swarm_tellos/share/swarm_tellos/launch/one.rviz'], output='screen'),

        # Joystick driver, generates /namespace/joy messages
        Node(package='joy', executable='joy_node', output='screen',
             name='joy_node', parameters=[{
                'use_sim_time': True,                           # Use /clock if available
            }]),

        Node(package='swarm_tellos', executable='global_controller.py', output='screen',arguments= drones ),

       
    ]

    # Per-drone entities
    for idx, namespace in enumerate(drones):
    
        suffix = '_' + str(idx + 1)
        urdf_path = os.path.join(tello_description_path, 'urdf', 'tello' + suffix + '.urdf')

        entities.extend([
        
            # Add a drone to the simulation
            Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
                 arguments=[urdf_path]+starting_locations[idx]),
            Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 arguments=[urdf_path]),
            Node(package='swarm_tellos', executable='drone_controller_gazebo.py', output='screen',arguments= [namespace]),

                 

        ])

    return LaunchDescription(entities)
