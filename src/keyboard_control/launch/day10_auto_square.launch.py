from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('keyboard_control'),
        'worlds',
        'day9_diff_drive.world'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/simple_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    auto = Node(
        package='keyboard_control',
        executable='auto_square_odom',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'odom_topic': '/model/simple_car/odometry',
            'side_length': 4.0,
            'linear_speed': 1.2,     # 更快
            'angular_speed': 1.4,    # 更快
            'yaw_tolerance_deg': 2.0,
            'dist_tolerance': 0.05,
            'rate_hz': 30.0,
        }]
    )

    return LaunchDescription([gazebo, bridge, auto])

