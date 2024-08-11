from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction,RegisterEventHandler
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    ld = LaunchDescription()



    ld.add_action(DeclareLaunchArgument(name='rviz-config-file-name', default_value='robot_nav.rviz', description='The file name of the rviz config file to use'))

    rviz_config_file = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config',LaunchConfiguration('rviz-config-file-name')])

    action_launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time' : False}]
    )
    



    ld.add_action(action_launch_rviz)
    return ld
