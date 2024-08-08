from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


    
def generate_launch_description():
    
    my_package_dir = get_package_share_directory('isaac_sim_demo')
    
    return LaunchDescription([

 
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    parameters=[{"Frame": "Drone1"}]
                ),
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    parameters=[{"Frame": "Drone2"}]
                ) ,
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    parameters=[{"Frame": "Drone3"}]
                ) ,
        Node(
                    package='isaac_sim_demo',
                    executable='tf_to_pos_node',
                    parameters=[{"Frame": "Drone4"}]
                )                 

    
    ])
