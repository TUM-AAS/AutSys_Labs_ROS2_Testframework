from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, Shutdown, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Locate other launch files
    controller_pkg_launch_dir = os.path.join(
        get_package_share_directory('controller_pkg'),
        'launch'
    )
    controller_launch_file = os.path.join(controller_pkg_launch_dir, 'controller.launch.py')

    pkg_share = get_package_share_directory('unity_bridge')
    unity_bin = os.path.join(pkg_share, 'unitysim', 'Linux_build_server.x86_64')

    vnav_sim_node = ExecuteProcess(
        cmd=[unity_bin]
    )

    traj_publisher_node = Node(
        package='unity_bridge',
        executable='traj_publisher',
        name='traj_publisher',
        output='screen',
        emulate_tty=True
    )

    w_to_unity_node = Node(
        package='unity_bridge',
        executable='w_to_unity',
        name='w_to_unity',
        output='screen',
        emulate_tty=True
    )

    unity_state_node = Node(
        package='unity_bridge',
        executable='unity_state',
        name='unity_state',
        output='screen',
        emulate_tty=True
    )

    # Include the controller_pkg launch file
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file)
    )

    # Test node (controller_test)
    test_node = Node(
        package='testframework',
        executable='controller_test',
        name='test_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
        # `cwd="node"` in ROS1 maps to working directory behavior in ROS2 → emulate_tty=True for proper stdout
    )

    time_limit = TimerAction(
        period=40.0,
        actions=[Shutdown(reason="Time limit reached.")]
    )

    return LaunchDescription([
        vnav_sim_node,
        traj_publisher_node,
        w_to_unity_node,
        unity_state_node,
        controller_launch,
        test_node,
        time_limit
    ])
