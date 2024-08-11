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

    urdf_launch_package = FindPackageShare('ezmower_robot')

    ld.add_action(DeclareLaunchArgument(name='jsp-choice', default_value='jsp', choices=['jsp-gui', 'jsp',],
                                        description='jsp mode : jsp-gui,jsp'))

 
    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                                        description='Should we use Gazebo time or ROS time ? '))
    


    should_use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition = IfCondition(
            EqualsSubstitution(LaunchConfiguration('jsp-choice'),"jsp")
        ),
        parameters=[{'use_sim_time' : should_use_sim_time}]
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("jsp-choice"),"jsp-gui")
        ),
        parameters=[{'use_sim_time' : should_use_sim_time}]
    ))



    return ld
