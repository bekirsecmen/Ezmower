from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo,ExecuteProcess,RegisterEventHandler,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()




    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        launch_arguments={
            'use_sim_time' : 'false',
            'slam_params_file': PathJoinSubstitution([FindPackageShare('ezmower_nav'), 'config', 'slam.yaml'])
            }.items()
    ))


    return ld