from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('zoomba')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'zoomba.urdf.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'view.rviz')
    use_sim_time = {'use_sim_time': True}
   
    # robot description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, {'robot_description': Command(['xacro ', xacro_file])}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[use_sim_time],
        arguments=["-d", rviz_config_file],
    )



    return LaunchDescription([
        robot_state_pub,
        rviz_node,

        ])