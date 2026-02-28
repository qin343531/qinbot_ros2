import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

'''
如果工程原来的qinbot_dis.launch.py运行后的gazebo环境无法加载出小车模型，可以试试这个工程
'''

def generate_launch_description():
    robot_name_in_model = 'ares_description'
    package_name = 'ares_description'
    urdf_name = "ares_description.urdf"
    world_name = "test.world"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, f'world/{world_name}')

    # 1. 启动 Gazebo server（保留正确配置）
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    # 2. 生成实体（直接生成，移除删除步骤，改用「先手动清理+唯一实体名」解决重复问题）
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', f'{robot_name_in_model}_{os.getpid()}',  # 新增：用进程ID做后缀，确保实体名唯一
            '-file', urdf_model_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # 3. 启动 robot_state_publisher
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_model_path],
            output='screen'
        )

    # 4. 启动 RVIZ（按需启用）
    start_rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    # 组装执行逻辑
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # 等待 Gazebo 启动 3 秒后生成实体
    ld.add_action(TimerAction(
        period=3.0,
        actions=[spawn_entity_cmd]
    ))
    # ld.add_action(start_rviz_cmd)

    return ld