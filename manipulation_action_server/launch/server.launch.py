import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import yaml

def generate_launch_description(): 

    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('manipulation_action_server')

    params_file = os.path.join(
        pkg_dir,
        'params',   
        'params.yaml'
    )


    mappings = {
        'arm': 'tiago-arm',
        'camera_model': 'orbbec-astra',
        'end_effector': 'pal-gripper',
        'ft_sensor': 'schunk-ft',
        'laser_model': 'sick-571',
        'wrist_model': 'wrist-2010'
    }

    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description(file_path=os.path.join(
            get_package_share_directory('tiago_description'),
            'robots', 'tiago.urdf.xacro'), mappings=mappings)
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory('tiago_mtc_examples'),
            'config', 'tiago_pal-gripper.srdf'))
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'kinematics_kdl.yaml'))
        .trajectory_execution(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'controllers', 'controllers_pal-gripper.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor({
                                    'publish_planning_scene': True,
                                    'publish_geometry_updates': True,
                                    'publish_state_updates': True,
                                    'publish_transforms_updates': True,
        })
        .pilz_cartesian_limits(file_path=os.path.join(
            get_package_share_directory('tiago_moveit_config'),
            'config', 'pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    server_cmd = Node(
        package='manipulation_action_server',
        executable='server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            move_group_capabilities,
            params_file
        ]
    )

    ld.add_action(server_cmd)
    return ld
