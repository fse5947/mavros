import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mavros_params_file = os.path.join(get_package_share_directory('mavros'),
                                      'config', 'mavros_params.yaml')

    return LaunchDescription([
        Node(package='mavros',
             executable='mavros_node',
             output='screen',
             emulate_tty=True,
             parameters=[mavros_params_file]),
    ])
