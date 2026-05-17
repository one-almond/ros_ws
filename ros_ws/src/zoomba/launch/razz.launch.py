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
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    use_sim_time = {'use_sim_time': False}
   
    lidar_driver_launch = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='log',
        parameters=[use_sim_time, os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'X4-Pro.yaml')],
    )

    diff_drive = Node(
        package='zoomba',
        executable='diff_drive.py',
        name='diff_drive_node',
        output='screen'
    )


    return LaunchDescription([
        lidar_driver_launch,
        diff_drive,

        ])