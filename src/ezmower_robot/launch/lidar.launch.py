from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()



    lidar_params_file = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','lidar.yaml'])

  


    action_launch_lidar = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[lidar_params_file]
    )
    






    ld.add_action(action_launch_lidar)

    return ld
