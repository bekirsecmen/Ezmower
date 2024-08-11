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
    # potential problem : sometimes we need to killall gzserver



    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ezmower_robot'), 'launch', 'rsp.launch.py']),
        launch_arguments={
            'use_sim_time' : 'false',
            'urdf_file_name': 'robot.urdf'
            }.items()
    ))


    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ezmower_robot'), 'launch', 'lidar.launch.py'])))


    rviz_config_file = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','robot.rviz'])

    action_launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time' : False}]
    )
    


    controller_params_file = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','my_controllers.yaml'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file,{'use_sim_time' : False}],
        remappings=[
            ('/controller_manager/robot_description','/robot_description'),
        ]
    )

    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['mobile_base_controller'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broadcaster'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # add the range sensor broadcasters

    range_sensor_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['range_sensor_broadcaster_left'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delayed_range_sensor_left_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[range_sensor_left_spawner],
        )
    )

    range_sensor_center_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['range_sensor_broadcaster_center'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delayed_range_sensor_center_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[range_sensor_center_spawner],
        )
    )

    range_sensor_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['range_sensor_broadcaster_right'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delayed_range_sensor_right_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[range_sensor_right_spawner],
        )
    )

    # add imu broadcaster

    imu_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['imu_broadcaster'],
        parameters=[{'use_sim_time' : False}],
        output="screen"
    )

    delayed_imu_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_spawner],
        )
    )

    # add ekf node

    robot_localization_yaml_path = PathJoinSubstitution([FindPackageShare('ezmower_robot'),'config','localization.yaml'])

    ekf_spawner = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_yaml_path],
        output="screen"
    )


    delayed_ekf_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ekf_spawner],
        )
    )


    delayed_rviz_launcher = TimerAction(period=6.0,actions=[action_launch_rviz])

    ld.add_action(delayed_controller_manager)
    ld.add_action(delay_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_range_sensor_left_spawner)
    ld.add_action(delayed_range_sensor_center_spawner)
    ld.add_action(delayed_range_sensor_right_spawner)
    #ld.add_action(delayed_imu_spawner)# IMU is broken :(
    ld.add_action(delayed_ekf_spawner)
    #ld.add_action(delayed_rviz_launcher)
    return ld
