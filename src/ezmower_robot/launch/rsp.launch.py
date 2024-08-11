from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

#
# Process XACRO and launch Robot State Publisher
#
def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('urdf_file_name',
                                        description='The name of the URDF file to launch'))
    ld.add_action(DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                                        description='Should we use Gazebo time or ROS time ? '))


    package_dir = FindPackageShare('ezmower_robot')
    urdf_path = PathJoinSubstitution([package_dir, 'urdf',LaunchConfiguration('urdf_file_name')])

    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    should_use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                          'use_sim_time' : should_use_sim_time,
                                          'publish_frequency' : 60.0,
                                          #'ignore_timestamp' : True,
                                      }])

    ld.add_action(robot_state_publisher_node)
    return ld
