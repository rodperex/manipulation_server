import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():

    pkg_dir = get_package_share_directory('moveto_action_control')

    params_file = os.path.join(pkg_dir, 'params', 'params.yaml')
    print('file: ', params_file)
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['moveto_action_client']['ros__parameters']
    
    ld = LaunchDescription()

    robot_cmd = Node(
        package='moveto_action_control',
        executable='client',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }, params]
    )

    ld.add_action(robot_cmd)
    return ld