#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Path to YAML with reference vertices
    reference_yaml = PathJoinSubstitution([
        FindPackageShare('testframework'),
        'config',
        'referenceVertex.yaml'
    ])

    # Included waypoint mission launch file (ROS2 version)
    waypoint_mission_launch = PathJoinSubstitution([
        FindPackageShare('basic_waypoint_pkg'),
        'launch',
        'waypoint_mission.launch.py'
    ])

    pkg_share = get_package_share_directory('unity_bridge')
    unity_bin = os.path.join(pkg_share, 'unitysim', 'Linux_build_server.x86_64')

    vnav_sim = ExecuteProcess(
        cmd=[unity_bin]
    )

    w_to_unity = Node(
        package='unity_bridge',
        executable='w_to_unity',
        name='w_to_unity',
        output='screen'
    )

    unity_state = Node(
        package='unity_bridge',
        executable='unity_state',
        name='unity_state',
        output='screen'
    )

    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    test_node = Node(
        package='testframework',
        executable='trajectory_test',
        name='test_node',
        output='screen',
        parameters=[reference_yaml],  # replaces <rosparam command="load" .../>
        on_exit=Shutdown()            # required="true" -> shut down if this node exits
    )

    waypoint_mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waypoint_mission_launch)
    )

    return LaunchDescription([
        vnav_sim,
        w_to_unity,
        unity_state,
        controller_node,
        test_node,
        waypoint_mission
    ])
