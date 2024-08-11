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

    ## Add our launch arguments 
    ld.add_action(
        DeclareLaunchArgument(
            'world_file',
            default_value= "barrels.world",
            description="The map to load while starting gazebo",
        ),
    )


    # potential problem : sometimes we need to killall gzserver, so just do it 
    killall_gzserver_action = ExecuteProcess(
        cmd=["killall","gzserver"],
        output="screen"
    )

    ld.add_action(killall_gzserver_action)

    # add the models dir to path so that gazebo can find our models

    models_dir_path =  PathJoinSubstitution([FindPackageShare('ezmower_sim'),'models'])

    ld.add_action(SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[models_dir_path]
    ))    

    world_path =  PathJoinSubstitution([FindPackageShare('ezmower_sim'),'worlds',LaunchConfiguration('world_file')])


    launch_gazebo_action = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'world' : world_path,
            }.items()
    )


    # run gzserver spawner after we killall gzserver
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=killall_gzserver_action,
            on_start=[launch_gazebo_action],
        )
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ezmower_robot'), 'launch', 'rsp.launch.py']),
        launch_arguments={
            'use_sim_time' : 'true',
            'urdf_file_name': 'robot_sim.urdf'
            }.items()
    ))


    rviz_config_file = PathJoinSubstitution([FindPackageShare('ezmower_sim'),'config','robot.rviz'])


    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time' : True}]
    ))


    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','my_bot'],
        output='screen'
    ))




    return ld