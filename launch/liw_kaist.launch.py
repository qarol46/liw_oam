from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Установка переменных окружения
    set_env_vars = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}][{time}][{name}]: {message}'
    )
    
    # Аргументы запуска
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='kaist.yaml',
        description='Configuration file name'
    )
    
    # Получение путей
    package_path = FindPackageShare('liw_oam')
    config_path = PathJoinSubstitution([package_path, 'config'])
    rviz_config_path = PathJoinSubstitution([package_path, 'rviz_cfg', 'visualization.rviz'])
    
    # Узел LIO Optimization
    lio_node = Node(
        package='liw_oam',
        executable='lio_optimization',
        name='lio_optimization',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, LaunchConfiguration('config_file')]),
            {
                'debug_output': False,
                'output_path': os.path.join(get_package_share_directory('liw_oam'), 'output'),
            }
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Узел RViz (условный)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        set_env_vars,
        rviz_arg,
        config_file_arg,
        lio_node,
        rviz_node,
    ])