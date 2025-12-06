import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 设置包名和 URDF 文件名
    package_name = 'fishbot_description'
    urdf_name = 'fishbot_gazebo.urdf'

    # 创建启动描述对象
    ld = LaunchDescription()

    # 查找功能包的路径
    pkg_share = get_package_share_directory(package_name)

    # 拼接出完整的 URDF 文件路径
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    # 创建 robot_state_publisher 节点，用于发布机器人的 TF 和状态
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]  # 传入 URDF 路径
    )

    # 创建 joint_state_publisher_gui 节点，显示关节状态 GUI，便于调试
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]  # 传入同一个 URDF 路径
    )

    # 创建 RViz2 节点，用于可视化机器人的状态
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'  # 输出显示在终端屏幕上
    )

    # 将所有节点添加到启动描述中
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz2_node)

    # 返回启动描述，ROS 2 会根据它来执行节点
    return ld
