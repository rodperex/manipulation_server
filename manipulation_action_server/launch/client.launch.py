import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    pkg_dir = get_package_share_directory('manipulation_action_server')

    # params_file = os.path.join(pkg_dir, 'params', 'params.yaml')
    # with open(params_file, 'r') as f:
    #     params = yaml.safe_load(f)['manipulation_action_server']['ros__parameters']

    ld = LaunchDescription()

    robot_cmd = Node(
        package='manipulation_action_server',
        executable='client',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    ld.add_action(robot_cmd)
    return ld
