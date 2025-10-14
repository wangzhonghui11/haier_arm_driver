from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ambot_driver_node",
            executable="ambot_node",
            name="ambot_driver_node",
            parameters=[os.path.join(
                    get_package_share_directory('ambot_driver_node'),
                    'config',
                    'n1_configurations.yaml'
                )]
        ),
    ])