import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_model_path = os.path.join(pkg_share, 'urdf', 'fishbot_gazebo.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fishbot.rviz')
    
    # 声明一些配置项
    # 仿真相关的启动配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 是否使用仿真时间（默认为 true）
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='如果为 true，使用 Gazebo 的仿真时间')

    # 是否启动 Gazebo 模拟器（默认为 true）
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='是否启动仿真器')

    # 启动 Gazebo 模拟器，加载指定的世界文件（使用绝对路径）
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': '/home/jie/gazebo_map/world/racemap.world'  # 使用绝对路径
        }.items()
    )

    # 向 Gazebo 中插入机器人模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'fishbot', '-file', urdf_model_path],
        output='screen'
    )

    # 启动 robot_state_publisher 节点，发布 TF 和机器人状态信息
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_model_path]
    )

    # 启动 RViz2，可视化机器人模型与传感器等信息
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # 创建并填充 LaunchDescription 对象
    ld = LaunchDescription()

    # 添加声明参数的动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)

    # 添加 Gazebo 启动相关命令
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)

    # 添加各个启动动作到 LaunchDescription 中
    ld.add_action(start_robot_state_publisher_cmd) # 发布 TF 和 URDF 状态
    ld.add_action(start_rviz_cmd) # 启动 RViz 可视化界面

    return ld
