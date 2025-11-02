from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 正确的 URDF 绝对路径（已验证）
    urdf_path = "/home/qin/car_ros2/install/ares_description/share/ares_description/urdf/ares_description.urdf"
    
    # 读取 URDF 文件
    try:
        with open(urdf_path, 'r') as f:
            robot_description = f.read()
        print(f"✅ URDF 加载成功！路径：{urdf_path}")
    except Exception as e:
        print(f"❌ URDF 加载失败！请检查路径：{urdf_path}")
        print(f"   错误：{e}")
        robot_description = ""

    return LaunchDescription([
        # 关节控制 GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # 机器人状态发布器（终于能拿到 URDF 了）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
