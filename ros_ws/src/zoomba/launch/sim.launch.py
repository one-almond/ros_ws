from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_dir = get_package_share_directory('zoomba')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'zoomba.urdf.xacro')
    robot_urdf = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'robot_description.urdf'
    )
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz.rviz')
    use_sim_time = {'use_sim_time': False}
   

    with open(robot_urdf, 'r') as infp:
        robot_description = infp.read()

    # robot description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, {'robot_description': robot_description}],
    )

    lidar_driver_launch = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='log',
        parameters=[use_sim_time, os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'X4-Pro.yaml')],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[use_sim_time],
        arguments=["-d", rviz_config_file],
    )

    slam_launch = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='log',
        parameters=[use_sim_time, os.path.join(get_package_share_directory('zoomba'), 'config', 'slam_params.yaml')],
    )

    rf2o_launch = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='log',
    )

    odom_pub_node = ExecuteProcess(
        cmd=['python3', '/home/user/ros_ws/src/zoomba/scripts/odom_pub.py'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_pub,
        #lidar_driver_launch,
        rviz_node,
        slam_launch,
        rf2o_launch,
        #odom_pub_node,

        ])