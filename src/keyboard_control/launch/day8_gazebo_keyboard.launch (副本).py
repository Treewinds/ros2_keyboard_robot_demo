from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取 world 文件路径
    # 确保你的包名是 keyboard_control，且 world 文件已安装
    world_path = os.path.join(
        get_package_share_directory('keyboard_control'),
        'worlds',
        'day9_diff_drive.world'
    )

    # 2. 启动 Gazebo (添加 -r 确保启动即运行物理引擎)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 3. ROS <-> GZ 桥接器
    # 将 Gazebo 的长路径话题重映射为 ROS 标准的 /cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 注意：这里建议去掉 @，直接用 [ 明确方向：ROS -> Gazebo
            '/model/simple_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/simple_car/cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    # 4. 你的键盘控制节点
    # 假设你的 Python 脚本在 setup.py 中注册的可执行文件名为 'keyboard_node'
    keyboard_control = Node(
            package='keyboard_control',
            executable='keyboard_car',
            output='screen',
            # 改为列表形式更稳定，-hold 可以让你在报错时看清日志
            prefix=['xterm -hold -e'], 
            emulate_tty=True
        )

    return LaunchDescription([
        gazebo,
        bridge,
        keyboard_control,
        LogInfo(msg="所有节点已启动。如果键盘控制窗口没弹出，请检查是否安装了 xterm (sudo apt install xterm)")
    ])
