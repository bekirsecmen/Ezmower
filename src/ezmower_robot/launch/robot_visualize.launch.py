from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,PythonExpression,EqualsSubstitution
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
    PathJoinSubstitution([FindPackageShare('ezmower_robot'), 'launch', 'rsp.launch.py']),
    launch_arguments={
        'use_sim_time' : 'false',
        'urdf_file_name': 'robot.urdf'
        }.items()
    ))


    ld.add_action( IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ezmower_robot'), 'launch', 'jsp.launch.py']),
        launch_arguments={
            'jsp-choice': 'jsp-gui',
            'use_sim_time' : 'false'
            }.items()
    ) )

    rviz_config_file = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','visualize.rviz'])


    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time' : True}]
    ))

    return ld