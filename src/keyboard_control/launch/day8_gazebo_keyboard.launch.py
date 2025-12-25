from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-r',
            '/opt/ros/jazzy/share/ros_gz_sim/worlds/diff_drive.sdf'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
    ])