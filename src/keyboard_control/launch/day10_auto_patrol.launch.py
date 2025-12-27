from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # world 路径（你已安装到 share/keyboard_control/worlds）
    world_path = os.path.join(
        get_package_share_directory('keyboard_control'),
        'worlds',
        'day9_diff_drive.world'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 只桥 /cmd_vel（你已经验证这条链路稳定）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/simple_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )


    # 自动巡航节点（发布 /cmd_vel）
    auto_patrol = Node(
        package='keyboard_control',
        executable='auto_patrol',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'linear_x': 0.5,
            'angular_z': 0.6,
            'forward_sec': 3.0,
            'turn_sec': 1.6,
        }],
    )

    return LaunchDescription([
        gazebo,
        bridge,
        auto_patrol
    ])
