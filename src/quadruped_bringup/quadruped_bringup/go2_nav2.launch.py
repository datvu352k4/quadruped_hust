import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/datvu/my_map.yaml', 
        description='Full path to the map file to load'
    )
    map_file_path = LaunchConfiguration('map')

    # Tham số TIME
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_params_file = '/home/datvu/quadruped_hust_ws/src/nav2.yaml'

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,  
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items()
    )

    rviz_cmd = [
        os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cmd[0]],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        map_file_arg,       
        use_sim_time_arg,    
        nav2_bringup_launch,
        rviz_node
    ])