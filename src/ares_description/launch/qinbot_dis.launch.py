import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'ares_description'
    package_name = 'ares_description'
    urdf_name = "ares_description.urdf"
    world_name = "test.world"
    # urdf_path = "/home/qin/car_ros2/install/ares_description/share/ares_description/urdf/ares_description.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, f'world/{world_name}')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')

    start_robot_state_publisher_cmd = Node( # 关节控制 GUI
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_model_path],
        )

    start_rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_rviz_cmd)


    return ld
    