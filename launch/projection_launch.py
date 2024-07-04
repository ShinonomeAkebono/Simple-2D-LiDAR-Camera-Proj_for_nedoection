import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(get_package_share_directory('lidar_camera_projection'),'urdf',urdf_file_name)
    with open(urdf,'r') as infp:
        robot_description = infp.read()
        
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='lidar_camera',
            output='screen',
            parameters=[{'robot_description':robot_description}],
            arguments=[urdf]
        ),
        Node(
            ## Configure the TF of the robot to the origin of the map coordinatesf
            package='lidar_camera_projection',
            executable='lidar_camera_projection_node'
        )
    ])