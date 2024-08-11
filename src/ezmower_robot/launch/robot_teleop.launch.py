from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo,ExecuteProcess,RegisterEventHandler,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(name='cmd-vel-topic', default_value='/mobile_base_controller/cmd_vel_unstamped', description='The cmd_vel topic to publish to'))
 
    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                                        description='Should we use Gazebo time or ROS time ? '))
    

    joy_params = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','joystick.yaml'])
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[joy_params]
    ))

    should_use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    ld.add_action(Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel',LaunchConfiguration('cmd-vel-topic'))]
    ))

    return ld