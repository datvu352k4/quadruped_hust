import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file_path = '/home/datvu/go2/urdf/go2_ros2.urdf'
    
    sim_script_path = '/home/datvu/quadruped_hust_ws/src/quadruped_bringup/scripts/go2_import.py'
    
    ctrl_script_path = '/home/datvu/quadruped_hust_ws/src/quadruped_bringup/scripts/go2_controller.py'

    joy_config_path = '/home/datvu/quadruped_hust_ws/src/joystick_config.yaml'

    robot_desc = ""
    try:
        with open(urdf_file_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        print(f"ERROR: URDF not found {urdf_file_path}")
     
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False, 
        }],
        remappings=[('/joint_states', '/go2/joint_states_custom')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'src/quadruped_bringup/config.rviz']
    )

    sim_process = ExecuteProcess(
        cmd=['python3','-u', sim_script_path],
        output='screen',
        name='genesis_sim'
    )

    ctrl_process = ExecuteProcess(
        cmd=['python3', ctrl_script_path],
        output='screen',
        name='rl_controller'
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')
        ),
        launch_arguments={
            'config_filepath': joy_config_path
        }.items()
    )
    return LaunchDescription([
        rsp_node,
        rviz_node,
        teleop_launch,
        sim_process,
        TimerAction(
            period=15.0, 
            actions=[ctrl_process]
        )
    ])